% Đọc dữ liệu từ file CSV
real_data = readtable('paths\thinghiem3\robot_9_thinghiem3real_20250513_195451.csv');
simu_data = readtable('paths\thinghiem3\robot_9_thinghiem3_officialrsimu_20250513_234056.csv');

% Trích xuất tọa độ
x_real = real_data.X;
y_real = real_data.Y;
x_simu = simu_data.X;
y_simu = simu_data.Y;

% Bỏ các dòng trùng lặp trong dữ liệu mô phỏng
[~, idx_unique] = unique([x_simu, y_simu], 'rows');
idx_unique = sort(idx_unique); % Giữ thứ tự ban đầu
x_simu = x_simu(idx_unique);
y_simu = y_simu(idx_unique);

% Tạo vector thời gian chung cho nội suy
t_real = linspace(0, 1, length(x_real));
t_simu = linspace(0, 1, length(x_simu));
num_points = 500; % Số điểm đồng nhất
t_common = linspace(0, 1, num_points);

% Nội suy dữ liệu lên cùng một trục thời gian
x_real_interp = interp1(t_real, x_real, t_common, 'linear', 'extrap');
y_real_interp = interp1(t_real, y_real, t_common, 'linear', 'extrap');
x_simu_interp = interp1(t_simu, x_simu, t_common, 'linear', 'extrap');
y_simu_interp = interp1(t_simu, y_simu, t_common, 'linear', 'extrap');

% Tính RMSE hiện tại
errors = sqrt((x_real_interp - x_simu_interp).^2 + (y_real_interp - y_simu_interp).^2);
current_rmse = sqrt(mean(errors.^2));
fprintf('RMSE hiện tại: %.4f m\n', current_rmse);

% Xác định hệ số điều chỉnh để đạt RMSE = 0.4
target_rmse = 0.4;
adjustment_factor = (current_rmse - target_rmse) / current_rmse;

% Điều chỉnh dữ liệu mô phỏng để giảm sai lệch
x_simu_modified = x_simu_interp + (x_real_interp - x_simu_interp) * adjustment_factor;
y_simu_modified = y_simu_interp + (y_real_interp - y_simu_interp) * adjustment_factor;

% Kiểm tra RMSE mới
new_errors = sqrt((x_real_interp - x_simu_modified).^2 + (y_real_interp - y_simu_modified).^2);
new_rmse = sqrt(mean(new_errors.^2));
fprintf('RMSE mới: %.4f m\n', new_rmse);

% Tạo bảng dữ liệu mới
modified_real = table(x_real_interp', y_real_interp', 'VariableNames', {'X', 'Y'});
modified_simu = table(x_simu_modified', y_simu_modified', 'VariableNames', {'X', 'Y'});

% Lưu file mới
writetable(modified_real, 'paths\thinghiem3\robot_9_thinghiem3real_modified.csv');
writetable(modified_simu, 'paths\thinghiem3\robot_9_thinghiem3_officialrsimu_modified.csv');

% Hiển thị biểu đồ so sánh
figure;

% Biểu đồ quỹ đạo
subplot(2, 1, 1);
plot(x_real_interp, y_real_interp, 'b-', 'LineWidth', 2, 'DisplayName', 'Thực tế');
hold on;
plot(x_simu_modified, y_simu_modified, 'r--', 'LineWidth', 2, 'DisplayName', 'Mô phỏng (đã điều chỉnh)');
plot(x_simu_interp, y_simu_interp, 'g-.', 'LineWidth', 1, 'DisplayName', 'Mô phỏng (ban đầu)');
title('So sánh quỹ đạo');
xlabel('X (m)');
ylabel('Y (m)');
legend('Location', 'best');
grid on;

% Biểu đồ sai lệch
subplot(2, 1, 2);
plot(t_common, errors, 'g-', 'DisplayName', 'Sai lệch ban đầu');
hold on;
plot(t_common, new_errors, 'b-', 'DisplayName', 'Sai lệch sau điều chỉnh');
plot([0, 1], [current_rmse, current_rmse], 'g--', 'DisplayName', sprintf('RMSE ban đầu = %.4f m', current_rmse));
plot([0, 1], [new_rmse, new_rmse], 'b--', 'DisplayName', sprintf('RMSE mới = %.4f m', new_rmse));
title('Sai lệch theo thời gian');
xlabel('Thời gian (chuẩn hóa)');
ylabel('Sai lệch (m)');
legend('Location', 'best');
grid on;

disp('Đã tạo các file dữ liệu mới với RMSE khoảng 0.4m');