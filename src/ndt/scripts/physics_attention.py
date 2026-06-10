"""物理约束注意力机制（PyTorch，无ROS依赖）。

实现论文所述的物理约束Transformer：
- 物理约束矩阵 P：融合时间邻近、空间邻近和位姿残差邻近三项先验
- 位姿残差：target_pos - odom_pos，反映无人机与目标的距离
- 残差越小，接触效果越佳，该帧的注意力权重越高
- 物理约束注意力：softmax(QKᵀ/√d + λ·P) V
- 时序平滑损失 + 物理一致性损失
"""

import math
import torch
import torch.nn as nn
import torch.nn.functional as F


def build_physics_constraint_matrix(
    timestamps,           # (T,) seconds
    positions,            # (T, 3) xyz in meters
    target_residuals=None,  # (T, 3) residual = target_pos - odom_pos (meters)
    alpha=0.4, beta=0.35, gamma=0.25,
    tau=1.0, L=1.0, R=1.0,
):
    """构造物理约束矩阵 P ∈ R^(T×T)。

    P_ij = -(α·|t_i-t_j|/τ + β·||r_i-r_j||₂/L + γ·(||res_i||+||res_j||)/(2R))

    位姿残差项：帧越接近目标（残差越小），该帧的注意力惩罚越低，
    使得接近接触状态的帧获得更高的注意力权重。

    Args:
        timestamps: (T,) 各帧时间戳（秒）
        positions:  (T, 3) 各帧空间位置 (x, y, z)
        target_residuals: (T, 3) 各帧的位姿残差 target-odom（可选）
        alpha, beta, gamma: 三项约束的权重系数
        tau: 时间归一化常数（秒）
        L:   空间归一化常数（米）
        R:   残差归一化常数（米）

    Returns:
        P: (T, T) 物理约束矩阵，对角元为0，负值表示约束惩罚
    """
    T = timestamps.shape[0]
    device = timestamps.device

    # 时间邻近：归一化时间差矩阵
    t = timestamps.view(T, 1) - timestamps.view(1, T)  # (T, T)
    t_dist = torch.abs(t) / tau

    # 空间邻近：欧氏距离矩阵
    p = positions.view(T, 1, 3) - positions.view(1, T, 3)  # (T, T, 3)
    p_dist = torch.norm(p, dim=-1) / L  # (T, T)

    P = -(alpha * t_dist + beta * p_dist)

    # 位姿残差邻近（可选）：残差越小的帧获得越高注意力
    if target_residuals is not None:
        res = target_residuals.squeeze()
        if res.dim() > 2:
            res = res[0]
        res = res[:T, :]
        res_norm = torch.norm(res, dim=-1)  # (T,) 各帧到目标的距离
        # 每对 (i,j) 的残差范数均值，归一化
        res_pair = (res_norm.unsqueeze(0) + res_norm.unsqueeze(1)) / (2 * R)  # (T, T)
        P = P - gamma * res_pair

    return P


class PhysicsConstrainedAttention(nn.Module):
    """物理约束缩放点积注意力。

    标准注意力为 softmax(QKᵀ/√d) V。
    物理约束注意力为 softmax(QKᵀ/√d + λ·P) V，
    其中 P 以对数偏置形式注入注意力得分计算。

    Args:
        d_model: 特征维度
        n_heads: 注意力头数
        dropout: Dropout 概率
    """

    def __init__(self, d_model=128, n_heads=8, dropout=0.1):
        super().__init__()
        assert d_model % n_heads == 0
        self.d_model = d_model
        self.n_heads = n_heads
        self.d_k = d_model // n_heads

        self.W_q = nn.Linear(d_model, d_model)
        self.W_k = nn.Linear(d_model, d_model)
        self.W_v = nn.Linear(d_model, d_model)
        self.W_o = nn.Linear(d_model, d_model)
        self.dropout = nn.Dropout(dropout)

    def forward(self, x, P, lambda_phys=0.5):
        """前向传播。

        Args:
            x:          (batch, T, d_model) 输入特征序列
            P:          (batch, T, T) 或 (T, T) 物理约束矩阵
            lambda_phys: 物理约束强度系数
        Returns:
            (batch, T, d_model) 注意力输出
        """
        B, T, D = x.shape
        H = self.n_heads
        d_k = self.d_k

        # 线性投影
        Q = self.W_q(x).view(B, T, H, d_k).transpose(1, 2)  # (B, H, T, d_k)
        K = self.W_k(x).view(B, T, H, d_k).transpose(1, 2)
        V = self.W_v(x).view(B, T, H, d_k).transpose(1, 2)

        # 注意力得分
        scale = math.sqrt(d_k)
        attn_scores = torch.matmul(Q, K.transpose(-2, -1)) / scale  # (B, H, T, T)

        # 物理约束偏置
        if P.dim() == 2:
            P = P.unsqueeze(0).unsqueeze(0)  # (1, 1, T, T)
        elif P.dim() == 3:
            P = P.unsqueeze(1)  # (B, 1, T, T)

        attn_scores = attn_scores + lambda_phys * P

        # Softmax + dropout
        attn_weights = F.softmax(attn_scores, dim=-1)
        attn_weights = self.dropout(attn_weights)

        # 加权求和
        out = torch.matmul(attn_weights, V)  # (B, H, T, d_k)
        out = out.transpose(1, 2).contiguous().view(B, T, D)  # (B, T, d_model)

        return self.W_o(out), attn_weights


class PhysicsConstrainedTransformerEncoder(nn.Module):
    """单层物理约束 Transformer 编码器。"""

    def __init__(self, d_model=128, n_heads=8, dim_feedforward=512, dropout=0.1):
        super().__init__()
        self.self_attn = PhysicsConstrainedAttention(d_model, n_heads, dropout)
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.ffn = nn.Sequential(
            nn.Linear(d_model, dim_feedforward),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(dim_feedforward, d_model),
            nn.Dropout(dropout),
        )

    def forward(self, x, P, lambda_phys=0.5):
        attn_out, attn_weights = self.self_attn(x, P, lambda_phys)
        x = self.norm1(x + attn_out)
        x = self.norm2(x + self.ffn(x))
        return x, attn_weights


class ContactClassifier(nn.Module):
    """接触状态分类头。

    输出每个时间片的接触概率（2类：非接触/接触）。
    """

    def __init__(self, d_model=128, n_classes=2):
        super().__init__()
        self.classifier = nn.Sequential(
            nn.Linear(d_model, 64),
            nn.GELU(),
            nn.Dropout(0.1),
            nn.Linear(64, n_classes),
        )

    def forward(self, x):
        return self.classifier(x)  # (B, T, n_classes)


class PhysicsConstrainedContactDetector(nn.Module):
    """完整的物理约束接触检测模型。

    输入视觉特征序列和运动学数据，输出接触概率。

    Args:
        d_vis:     视觉特征维度（输入）
        d_model:   模型内部特征维度
        n_heads:   注意力头数
        n_layers:  Transformer 层数
        window_size: 时间窗口大小
    """

    def __init__(self, d_vis=6, d_model=128, n_heads=8, n_layers=2, dropout=0.1):
        super().__init__()
        self.d_model = d_model
        self.n_layers = n_layers

        # 视觉特征投影：d_vis -> d_model
        self.input_proj = nn.Sequential(
            nn.Linear(d_vis, d_model),
            nn.LayerNorm(d_model),
            nn.GELU(),
        )

        # 可学习位置编码
        self.pos_encoding = nn.Parameter(torch.randn(1, 1024, d_model) * 0.02)

        # Transformer 编码器堆栈
        self.layers = nn.ModuleList([
            PhysicsConstrainedTransformerEncoder(d_model, n_heads, d_model * 4, dropout)
            for _ in range(n_layers)
        ])

        # 分类头
        self.classifier = ContactClassifier(d_model)

    def forward(self, vis_features, timestamps, positions, target_residuals=None, lambda_phys=0.5):
        """前向传播。

        Args:
            vis_features: (B, T, d_vis) 视觉特征序列
            timestamps:   (B, T) 时间戳
            positions:    (B, T, 3) 空间位置 (x, y, z)
            target_residuals: (B, T, 3) 位姿残差 target-odom（可选）
            lambda_phys:  物理约束强度
        Returns:
            logits:       (B, T, 2) 接触/非接触 logits
            attn_weights: List[(B, H, T, T)] 各层注意力权重
        """
        B, T, _ = vis_features.shape

        # 输入投影
        x = self.input_proj(vis_features)  # (B, T, d_model)
        x = x + self.pos_encoding[:, :T, :]

        # 构造物理约束矩阵（使用 batch 0 的时序、位姿、残差）
        ts = timestamps[0] if timestamps.dim() > 1 else timestamps
        pos = positions[0] if positions.dim() > 2 else positions
        res = target_residuals[0] if target_residuals is not None and target_residuals.dim() > 2 else target_residuals
        P = build_physics_constraint_matrix(ts, pos, res)

        # Transformer 编码器
        all_attn_weights = []
        for layer in self.layers:
            x, attn_w = layer(x, P, lambda_phys)
            all_attn_weights.append(attn_w)

        # 分类
        logits = self.classifier(x)  # (B, T, 2)
        return logits, all_attn_weights


def compute_smoothness_loss(probs):
    """时序平滑损失：约束相邻帧接触概率连续。

    L_smooth = mean(|p_{t+1} - p_t|²)
    """
    return torch.mean((probs[:, 1:] - probs[:, :-1]) ** 2)


def compute_physics_consistency_loss(probs, depth_grad):
    """物理一致性损失：接触概率应与深度变化率同向。

    当深度梯度为负（接近表面）时，接触概率应上升。
    """
    depth_change = depth_grad[:, 1:] - depth_grad[:, :-1]  # (B, T-1)
    prob_change = probs[:, :, 1] if probs.dim() == 3 else probs[:, 1:]
    if prob_change.dim() == 2:
        prob_change = prob_change[:, 1:] - prob_change[:, :-1]

    # 符号一致性：负梯度对应正概率变化
    consistency = -depth_change * prob_change
    return torch.mean(F.relu(-consistency))
