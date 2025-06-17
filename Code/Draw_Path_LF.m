function Draw_Path_LF()
% Hàm chính để phân tích và vẽ quỹ đạo Leader-Follower

% Đặt đơn vị đo mặc định
unit = 'm';

% Chọn dữ liệu của các robot
[x_leader_real_full, y_leader_real_full, x_leader_simu_full, y_leader_simu_full, ...
 x_follower_real_full, y_follower_real_full, x_follower_simu_full, y_follower_simu_full] = selectRobotData();

% Tính toán khoảng cách giữa Leader và Follower (thực tế)
leader_follower_real_dists = sqrt((x_leader_real_full - x_follower_real_full).^2 + ...
                                 (y_leader_real_full - y_follower_real_full).^2);

% Tính toán khoảng cách giữa Leader và Follower (mô phỏng)
leader_follower_simu_dists = sqrt((x_leader_simu_full - x_follower_simu_full).^2 + ...
                                 (y_leader_simu_full - y_follower_simu_full).^2);

% Yêu cầu người dùng nhập tổng thời gian thực hiện
prompt = {'Nhập tổng thời gian thực hiện quỹ đạo thực tế (giây):','Nhập tổng thời gian thực hiện quỹ đạo mô phỏng (giây):'};
dlgtitle = 'Thông tin thời gian';
dims = [1 50];
definput = {'30','30'}; % Giá trị mặc định
answer = inputdlg(prompt,dlgtitle,dims,definput);

if isempty(answer)
    % Người dùng đã hủy
    return;
end

% Lấy thời gian từ input
real_total_time = str2double(answer{1});
simu_total_time = str2double(answer{2});

% Kiểm tra đầu vào hợp lệ
if isnan(real_total_time) || isnan(simu_total_time) || real_total_time <= 0 || simu_total_time <= 0
    errordlg('Thời gian không hợp lệ! Vui lòng nhập số dương.', 'Lỗi');
    return;
end

% Tính thời gian cho mỗi điểm quỹ đạo
leader_real_time_step = real_total_time / length(x_leader_real_full);
follower_real_time_step = real_total_time / length(x_follower_real_full);
leader_simu_time_step = simu_total_time / length(x_leader_simu_full);
follower_simu_time_step = simu_total_time / length(x_follower_simu_full);

% Tạo vectors thời gian
t_leader_real = (0:length(x_leader_real_full)-1) * leader_real_time_step;
t_follower_real = (0:length(x_follower_real_full)-1) * follower_real_time_step;
t_leader_simu = (0:length(x_leader_simu_full)-1) * leader_simu_time_step;
t_follower_simu = (0:length(x_follower_simu_full)-1) * follower_simu_time_step;

% Tạo vectors thời gian cho khoảng cách Leader-Follower
t_dist_real = linspace(0, real_total_time, length(leader_follower_real_dists));
t_dist_simu = linspace(0, simu_total_time, length(leader_follower_simu_dists));

% Tạo vector thời gian chung cho phân tích sai lệch
min_time = 0;
max_time = min(real_total_time, simu_total_time);
num_points = 500; % Số điểm nội suy
t_common_uniform = linspace(min_time, max_time, num_points);

% Nội suy quỹ đạo Leader thực tế và mô phỏng vào vector thời gian chung
x_leader_real_interp = interp1(t_leader_real, x_leader_real_full, t_common_uniform, 'linear', 'extrap');
y_leader_real_interp = interp1(t_leader_real, y_leader_real_full, t_common_uniform, 'linear', 'extrap');
x_leader_simu_interp = interp1(t_leader_simu, x_leader_simu_full, t_common_uniform, 'linear', 'extrap');
y_leader_simu_interp = interp1(t_leader_simu, y_leader_simu_full, t_common_uniform, 'linear', 'extrap');

% Nội suy quỹ đạo Follower thực tế và mô phỏng vào vector thời gian chung
x_follower_real_interp = interp1(t_follower_real, x_follower_real_full, t_common_uniform, 'linear', 'extrap');
y_follower_real_interp = interp1(t_follower_real, y_follower_real_full, t_common_uniform, 'linear', 'extrap');
x_follower_simu_interp = interp1(t_follower_simu, x_follower_simu_full, t_common_uniform, 'linear', 'extrap');
y_follower_simu_interp = interp1(t_follower_simu, y_follower_simu_full, t_common_uniform, 'linear', 'extrap');

% Tính sai lệch cho Leader và Follower
delta_x_leader = x_leader_simu_interp - x_leader_real_interp;
delta_y_leader = y_leader_simu_interp - y_leader_real_interp;
leader_errors = sqrt(delta_x_leader.^2 + delta_y_leader.^2);

delta_x_follower = x_follower_simu_interp - x_follower_real_interp;
delta_y_follower = y_follower_simu_interp - y_follower_real_interp;
follower_errors = sqrt(delta_x_follower.^2 + delta_y_follower.^2);

% Tạo figure cho biểu đồ
fig = figure('Name', 'Phân tích quỹ đạo Leader-Follower', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

% 1. Subplot: Quỹ đạo tổng quan (loại bỏ vectors sai lệch)
subplot(2, 1, 1);
hold on;

% Vẽ quỹ đạo Leader
plot(x_leader_real_full, y_leader_real_full, '-b', 'LineWidth', 2.5, 'DisplayName', 'Leader Thực tế');
plot(x_leader_simu_full, y_leader_simu_full, '--b', 'LineWidth', 2, 'DisplayName', 'Leader Mô phỏng');

% Vẽ quỹ đạo Follower
plot(x_follower_real_full, y_follower_real_full, '-r', 'LineWidth', 2, 'DisplayName', 'Follower Thực tế');
plot(x_follower_simu_full, y_follower_simu_full, '--r', 'LineWidth', 1.5, 'DisplayName', 'Follower Mô phỏng');

% Đánh dấu điểm bắt đầu và kết thúc
plot(x_leader_real_full(1), y_leader_real_full(1), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Leader Start');
plot(x_leader_real_full(end), y_leader_real_full(end), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Leader End');
plot(x_follower_real_full(1), y_follower_real_full(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Follower Start');
plot(x_follower_real_full(end), y_follower_real_full(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Follower End');

% Đã loại bỏ phần vẽ vector sai lệch cho Leader và Follower

title('Tổng quan quỹ đạo Leader-Follower');
xlabel(['Tọa độ X (', unit, ')']);
ylabel(['Tọa độ Y (', unit, ')']);
grid on;
axis equal;
legend('Location', 'best', 'NumColumns', 2);

% Tạo figure mới cho biểu đồ sai lệch Leader
fig_leader_error = figure('Name', 'Sai lệch quỹ đạo Leader', 'NumberTitle', 'off', 'Position', [150, 150, 1000, 500]);

% Tính toán sai lệch khoảng cách cho Leader
leader_errors = sqrt(delta_x_leader.^2 + delta_y_leader.^2);

% Tính các thông số thống kê
mean_leader_error = mean(leader_errors);
std_leader_error = std(leader_errors);
max_leader_error = max(leader_errors);
min_leader_error = min(leader_errors);
rmse_leader = sqrt(mean(leader_errors.^2));

% Vẽ biểu đồ sai lệch theo thời gian
subplot(2, 1, 1);
plot(t_common_uniform, leader_errors, '-b', 'LineWidth', 1.5);
hold on;

% Vẽ các đường tham chiếu thống kê
plot([t_common_uniform(1), t_common_uniform(end)], [mean_leader_error, mean_leader_error], '--r', 'LineWidth', 1.5);
plot([t_common_uniform(1), t_common_uniform(end)], [rmse_leader, rmse_leader], '-.g', 'LineWidth', 1.5);

% Thêm vùng hiển thị độ lệch chuẩn
x_range = [t_common_uniform, fliplr(t_common_uniform)];
y_range = [mean_leader_error + std_leader_error*ones(size(t_common_uniform)), ...
           fliplr(mean_leader_error - std_leader_error*ones(size(t_common_uniform)))];
fill(x_range, y_range, 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% Cài đặt các thuộc tính biểu đồ
grid on;
title(sprintf('Sai lệch khoảng cách Leader theo thời gian (RMSE = %.4f %s)', rmse_leader, unit));
xlabel('Thời gian (giây)');
ylabel(['Sai lệch (', unit, ')']);
legend('Sai lệch tức thời', 'Sai lệch trung bình', 'RMSE', '±1 độ lệch chuẩn', 'Location', 'best');

% Tạo figure mới cho biểu đồ sai lệch Follower
fig_follower_error = figure('Name', 'Sai lệch quỹ đạo Follower', 'NumberTitle', 'off', 'Position', [200, 200, 1000, 500]);

% Tính toán sai lệch khoảng cách cho Follower (đã tính trước đó)
% follower_errors = sqrt(delta_x_follower.^2 + delta_y_follower.^2);

% Tính các thông số thống kê
mean_follower_error = mean(follower_errors);
std_follower_error = std(follower_errors);
max_follower_error = max(follower_errors);
min_follower_error = min(follower_errors);
rmse_follower = sqrt(mean(follower_errors.^2));

% Vẽ biểu đồ sai lệch theo thời gian
subplot(2, 1, 1);
plot(t_common_uniform, follower_errors, '-r', 'LineWidth', 1.5);
hold on;

% Vẽ các đường tham chiếu thống kê
plot([t_common_uniform(1), t_common_uniform(end)], [mean_follower_error, mean_follower_error], '--b', 'LineWidth', 1.5);
plot([t_common_uniform(1), t_common_uniform(end)], [rmse_follower, rmse_follower], '-.g', 'LineWidth', 1.5);

% Thêm vùng hiển thị độ lệch chuẩn
x_range = [t_common_uniform, fliplr(t_common_uniform)];
y_range = [mean_follower_error + std_follower_error*ones(size(t_common_uniform)), ...
           fliplr(mean_follower_error - std_follower_error*ones(size(t_common_uniform)))];
fill(x_range, y_range, 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% Cài đặt các thuộc tính biểu đồ
grid on;
title(sprintf('Sai lệch khoảng cách Follower theo thời gian (RMSE = %.4f %s)', rmse_follower, unit));
xlabel('Thời gian (giây)');
ylabel(['Sai lệch (', unit, ')']);
legend('Sai lệch tức thời', 'Sai lệch trung bình', 'RMSE', '±1 độ lệch chuẩn', 'Location', 'best');

% Vẽ biểu đồ phân bố sai lệch
subplot(2, 1, 2);
histogram(follower_errors, min(30, round(sqrt(length(follower_errors)))), 'Normalization', 'probability', 'FaceColor', 'r');
hold on;
xline(mean_follower_error, '--b', 'LineWidth', 1.5);
xline(rmse_follower, '-.g', 'LineWidth', 1.5);

% Tính toán phân bố chuẩn lý thuyết
x_range = linspace(0, max(follower_errors)*1.2, 100);
pd = fitdist(follower_errors(:), 'Normal');  % Chuyển thành vector cột để tránh lỗi
y_normal = pdf(pd, x_range);
plot(x_range, y_normal, 'b-', 'LineWidth', 1.5);

% Cài đặt các thuộc tính biểu đồ
grid on;
title('Phân bố sai lệch khoảng cách Follower');
xlabel(['Sai lệch (', unit, ')']);
ylabel('Tần suất');
legend('Phân bố thực tế', 'Trung bình', 'RMSE', 'Đường cong phân bố chuẩn', 'Location', 'best');

% Hiển thị thống kê
annotation('textbox', [0.15, 0.02, 0.7, 0.05], 'String', ...
    sprintf(['Thống kê sai lệch Follower: Trung bình = %.4f %s, Độ lệch chuẩn = %.4f %s, ' ...
             'Min = %.4f %s, Max = %.4f %s, RMSE = %.4f %s'], ...
    mean_follower_error, unit, std_follower_error, unit, ...
    min_follower_error, unit, max_follower_error, unit, rmse_follower, unit), ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 9);

% Chỉnh sửa layout
sgtitle('Phân tích sai lệch khoảng cách Follower (Thực tế vs Mô phỏng)', 'FontSize', 14, 'FontWeight', 'bold');

% Tạo biểu đồ so sánh Leader và Follower
fig_compare = figure('Name', 'So sánh sai lệch Leader-Follower', 'NumberTitle', 'off', 'Position', [250, 250, 1000, 400]);
plot(t_common_uniform, leader_errors, '-b', 'LineWidth', 1.5, 'DisplayName', 'Sai lệch Leader');
hold on;
plot(t_common_uniform, follower_errors, '-r', 'LineWidth', 1.5, 'DisplayName', 'Sai lệch Follower');
plot([t_common_uniform(1), t_common_uniform(end)], [rmse_leader, rmse_leader], '--b', 'LineWidth', 1, 'DisplayName', 'RMSE Leader');
plot([t_common_uniform(1), t_common_uniform(end)], [rmse_follower, rmse_follower], '--r', 'LineWidth', 1, 'DisplayName', 'RMSE Follower');
grid on;
title(sprintf('So sánh sai lệch: RMSE Leader = %.4f %s, RMSE Follower = %.4f %s', rmse_leader, unit, rmse_follower, unit));
xlabel('Thời gian (giây)');
ylabel(['Sai lệch (', unit, ')']);
legend('Location', 'best');
end % End of the main function

% Định nghĩa hàm selectRobotData
function [x_leader_real_full, y_leader_real_full, x_leader_simu_full, y_leader_simu_full, ...
          x_follower_real_full, y_follower_real_full, x_follower_simu_full, y_follower_simu_full] = selectRobotData()

% Hiển thị hướng dẫn
msgbox('Bạn sẽ được yêu cầu chọn 4 file dữ liệu: Leader thực tế, Leader mô phỏng, Follower thực tế và Follower mô phỏng.');

% Chọn file cho Leader thực tế
[file_leader_real, path_leader_real] = uigetfile('*.csv', 'Chọn file dữ liệu Leader thực tế');
if isequal(file_leader_real, 0)
    error('Đã hủy việc chọn file Leader thực tế.');
end
leader_real_path = fullfile(path_leader_real, file_leader_real);

% Chọn file cho Leader mô phỏng
[file_leader_simu, path_leader_simu] = uigetfile('*.csv', 'Chọn file dữ liệu Leader mô phỏng');
if isequal(file_leader_simu, 0)
    error('Đã hủy việc chọn file Leader mô phỏng.');
end
leader_simu_path = fullfile(path_leader_simu, file_leader_simu);

% Chọn file cho Follower thực tế
[file_follower_real, path_follower_real] = uigetfile('*.csv', 'Chọn file dữ liệu Follower thực tế');
if isequal(file_follower_real, 0)
    error('Đã hủy việc chọn file Follower thực tế.');
end
follower_real_path = fullfile(path_follower_real, file_follower_real);

% Chọn file cho Follower mô phỏng
[file_follower_simu, path_follower_simu] = uigetfile('*.csv', 'Chọn file dữ liệu Follower mô phỏng');
if isequal(file_follower_simu, 0)
    error('Đã hủy việc chọn file Follower mô phỏng.');
end
follower_simu_path = fullfile(path_follower_simu, file_follower_simu);

% Đọc dữ liệu từ các file
try
    % Leader thực tế
    leader_real_data = readtable(leader_real_path);
    x_leader_real_full = leader_real_data.X;
    y_leader_real_full = leader_real_data.Y;
    
    % Leader mô phỏng
    leader_simu_data = readtable(leader_simu_path);
    x_leader_simu_full = leader_simu_data.X;
    y_leader_simu_full = leader_simu_data.Y;
    
    % Follower thực tế
    follower_real_data = readtable(follower_real_path);
    x_follower_real_full = follower_real_data.X;
    y_follower_real_full = follower_real_data.Y;
    
    % Follower mô phỏng
    follower_simu_data = readtable(follower_simu_path);
    x_follower_simu_full = follower_simu_data.X;
    y_follower_simu_full = follower_simu_data.Y;
    
catch ME
    errordlg(['Lỗi khi đọc file: ' ME.message], 'Lỗi');
    rethrow(ME);
end

% Hiển thị thông tin về dữ liệu
fprintf('Đã đọc dữ liệu từ files:\n');
fprintf('Leader thực tế: %s (%d điểm)\n', file_leader_real, length(x_leader_real_full));
fprintf('Leader mô phỏng: %s (%d điểm)\n', file_leader_simu, length(x_leader_simu_full));
fprintf('Follower thực tế: %s (%d điểm)\n', file_follower_real, length(x_follower_real_full));
fprintf('Follower mô phỏng: %s (%d điểm)\n', file_follower_simu, length(x_follower_simu_full));
end % End of selectRobotData function

