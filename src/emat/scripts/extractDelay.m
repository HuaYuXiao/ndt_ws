function d = extractDelay(data)
    data_vector = data(:);
    if length(data_vector) < 5000
        error('extractDelay:InsufficientData', '数据长度至少需要 5000 个点，当前为 %d', length(data_vector));
    end
    data_vector = data_vector(200:5000);   % 选取200到5000的数据点
    data_vector = data_vector - 127;

    % 希尔伯特变换获取包络
    analytic_signal = hilbert(data_vector);
    envelope = abs(analytic_signal);

    % 对包络信号进行低通滤波
    % 假设采样频率为Fs（您需要根据实际数据设置）
    Fs = 1000000;  % 请根据数据实际情况修改采样频率
    % 设置截止频率（通常远低于原始信号的频率）
    cutoff_freq = 10;  % Hz，根据您的包络变化频率调整
    filtered_envelope = lowpass(envelope, cutoff_freq, Fs);
    
    plot(filtered_envelope);
    hold on;

    % 找出滤波后包络的峰值点（比前一个点高且比后一个点高）
    peak_indices = find(filtered_envelope(2:end-1) > filtered_envelope(1:end-2) & ...
                        filtered_envelope(2:end-1) > filtered_envelope(3:end)) + 1;

    % 获取峰值点的坐标
    peak_x = peak_indices;
    peak_y = filtered_envelope(peak_indices);

    % 找到幅值最大的两个峰值点
    [sorted_peak_y, sorted_indices] = sort(peak_y, 'descend');
    top_two_indices = sorted_indices(1:min(2, length(sorted_indices)));  % 确保至少有2个峰值点

    % 提取最大两个峰值点的信息
    top_two_peak_x = peak_x(top_two_indices);
    top_two_peak_y = peak_y(top_two_indices);

    % 计算横坐标差值
    if length(top_two_peak_x) >= 2
        x_difference = abs(top_two_peak_x(1) - top_two_peak_x(2));
        d = x_difference/40000000*3240/2*1000;
    else
        d = NaN;
    end
end