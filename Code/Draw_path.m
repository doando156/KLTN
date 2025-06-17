% filepath: c:\Users\Do Doan\MATLAB\Projects\Simulation_real_system\Draw_path.m
% Mở hộp thoại để người dùng chọn file quỹ đạo thực tế
[real_file, real_path] = uigetfile('*.csv', 'Chọn file quỹ đạo thực tế');
if isequal(real_file, 0)
    disp('Không chọn file quỹ đạo thực tế.');
    return;
end

% Mở hộp thoại để người dùng chọn file quỹ đạo mô phỏng
[simu_file, simu_path] = uigetfile('*.csv', 'Chọn file quỹ đạo mô phỏng');
if isequal(simu_file, 0)
    disp('Không chọn file quỹ đạo mô phỏng.');
    return;
end

% Đọc dữ liệu từ các file đã chọn
try
    real_data = readtable(fullfile(real_path, real_file)); 
    simu_data = readtable(fullfile(simu_path, simu_file));
catch e
    errordlg(['Lỗi khi đọc file: ', e.message], 'Lỗi');
    return;
end

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

% Kiểm tra cấu trúc dữ liệu và cột
try
    % Xử lý tên cột có thể khác nhau
    real_cols = real_data.Properties.VariableNames;
    simu_cols = simu_data.Properties.VariableNames;
    
    % Tìm cột X và Y (giả sử chữ cái đầu tiên của tên cột là X hoặc Y)
    x_real_col = real_cols(strcmpi(real_cols, 'X') | ...
                         strcmpi(real_cols, 'x') | ...
                         contains(lower(real_cols), 'x_'));
    y_real_col = real_cols(strcmpi(real_cols, 'Y') | ...
                         strcmpi(real_cols, 'y') | ...
                         contains(lower(real_cols), 'y_'));
    
    x_simu_col = simu_cols(strcmpi(simu_cols, 'X') | ...
                         strcmpi(simu_cols, 'x') | ...
                         contains(lower(simu_cols), 'x_'));
    y_simu_col = simu_cols(strcmpi(simu_cols, 'Y') | ...
                         strcmpi(simu_cols, 'y') | ...
                         contains(lower(simu_cols), 'y_'));
    
    % Lấy dữ liệu từ các cột đã tìm
    x_real = real_data.(x_real_col{1});
    y_real = real_data.(y_real_col{1});
    x_simu = simu_data.(x_simu_col{1});
    y_simu = simu_data.(y_simu_col{1});
catch
    % Nếu không tìm được cột phù hợp, thử dùng các cột thứ 1 và 2
    try
        x_real = real_data{:,1};
        y_real = real_data{:,2};
        x_simu = simu_data{:,1};
        y_simu = simu_data{:,2};
        
        warning('Không tìm thấy cột X/Y, sử dụng cột 1 và 2');
    catch
        errordlg('Không thể xác định dữ liệu tọa độ X và Y trong file.', 'Lỗi');
        return;
    end
end

% Đảm bảo dữ liệu là dạng số
x_real = double(x_real);
y_real = double(y_real);
x_simu = double(x_simu);
y_simu = double(y_simu);

% Lưu trữ dữ liệu gốc để vẽ đồ thị đầy đủ
x_real_full = x_real;
y_real_full = y_real;
x_simu_full = x_simu;
y_simu_full = y_simu;

% Tính thời gian giữa các điểm
real_time_step = real_total_time / length(x_real_full);
simu_time_step = simu_total_time / length(x_simu_full);

% Tạo trục thời gian cho dữ liệu gốc
t_real_full = (0:length(x_real_full)-1) * real_time_step;
t_simu_full = (0:length(x_simu_full)-1) * simu_time_step;

% Hiển thị thông tin về bước thời gian
fprintf('Bước thời gian quỹ đạo thực tế: %.4f giây (%.2f Hz)\n', real_time_step, 1/real_time_step);
fprintf('Bước thời gian quỹ đạo mô phỏng: %.4f giây (%.2f Hz)\n', simu_time_step, 1/simu_time_step);

% Xử lý nếu độ dài quỹ đạo khác nhau - sử dụng phương pháp nội suy
len_real = length(x_real);
len_simu = length(x_simu);

if len_real ~= len_simu
    fprintf('Độ dài quỹ đạo khác nhau (Thực tế: %d, Mô phỏng: %d).\n', len_real, len_simu);
    
    % Hỏi người dùng muốn nội suy theo quỹ đạo dài hay ngắn
    choice = questdlg(['Quỹ đạo có độ dài khác nhau. Chọn phương pháp nội suy:'], ...
                  'Chọn phương pháp nội suy', ...
                  'Nội suy theo quỹ đạo dài', 'Nội suy theo quỹ đạo ngắn', 'Cắt ngắn dữ liệu', 'Nội suy theo quỹ đạo dài');
              
    switch choice
        case 'Nội suy theo quỹ đạo dài'
            if len_real > len_simu
                % Nội suy quỹ đạo mô phỏng để khớp với độ dài quỹ đạo thực tế (dài hơn)
                t_simu = linspace(0, 1, len_simu);
                t_interp = linspace(0, 1, len_real);
                x_simu_comp = interp1(t_simu, x_simu, t_interp, 'pchip');
                y_simu_comp = interp1(t_simu, y_simu, t_interp, 'pchip');
                x_real_comp = x_real;
                y_real_comp = y_real;
                fprintf('Đã nội suy quỹ đạo mô phỏng từ %d lên %d điểm để khớp với quỹ đạo thực tế.\n', len_simu, len_real);
            else
                % Nội suy quỹ đạo thực tế để khớp với độ dài quỹ đạo mô phỏng (dài hơn)
                t_real = linspace(0, 1, len_real);
                t_interp = linspace(0, 1, len_simu);
                x_real_comp = interp1(t_real, x_real, t_interp, 'pchip');
                y_real_comp = interp1(t_real, y_real, t_interp, 'pchip');
                x_simu_comp = x_simu;
                y_simu_comp = y_simu;
                fprintf('Đã nội suy quỹ đạo thực tế từ %d lên %d điểm để khớp với quỹ đạo mô phỏng.\n', len_real, len_simu);
            end
            
        case 'Nội suy theo quỹ đạo ngắn'
            % Tái lấy mẫu cả hai quỹ đạo với số điểm bằng với quỹ đạo ngắn nhất
            min_len = min(len_real, len_simu);
            t_real = linspace(0, 1, len_real);
            t_simu = linspace(0, 1, len_simu);
            t_common = linspace(0, 1, min_len);
            
            x_real_comp = interp1(t_real, x_real, t_common, 'pchip');
            y_real_comp = interp1(t_real, y_real, t_common, 'pchip');
            x_simu_comp = interp1(t_simu, x_simu, t_common, 'pchip');
            y_simu_comp = interp1(t_simu, y_simu, t_common, 'pchip');
            fprintf('Đã nội suy cả hai quỹ đạo xuống %d điểm.\n', min_len);
            
        case 'Cắt ngắn dữ liệu'
            % Phương pháp cắt ngắn như cũ
            min_len = min(len_real, len_simu);
            x_real_comp = x_real(1:min_len);
            y_real_comp = y_real(1:min_len);
            x_simu_comp = x_simu(1:min_len);
            y_simu_comp = y_simu(1:min_len);
            fprintf('Đã cắt ngắn dữ liệu để tính sai lệch xuống %d điểm.\n', min_len);
            
        otherwise
            % Mặc định: nội suy theo quỹ đạo dài
            if len_real > len_simu
                t_simu = linspace(0, 1, len_simu);
                t_interp = linspace(0, 1, len_real);
                x_simu_comp = interp1(t_simu, x_simu, t_interp, 'pchip');
                y_simu_comp = interp1(t_simu, y_simu, t_interp, 'pchip');
                x_real_comp = x_real;
                y_real_comp = y_real;
                fprintf('Đã nội suy quỹ đạo mô phỏng từ %d lên %d điểm để khớp với quỹ đạo thực tế.\n', len_simu, len_real);
            else
                t_real = linspace(0, 1, len_real);
                t_interp = linspace(0, 1, len_simu);
                x_real_comp = interp1(t_real, x_real, t_interp, 'pchip');
                y_real_comp = interp1(t_real, y_real, t_interp, 'pchip');
                x_simu_comp = x_simu;
                y_simu_comp = y_simu;
                fprintf('Đã nội suy quỹ đạo thực tế từ %d lên %d điểm để khớp với quỹ đạo mô phỏng.\n', len_real, len_simu);
            end
    end
else
    % Nếu độ dài bằng nhau, sử dụng toàn bộ dữ liệu
    x_real_comp = x_real;
    y_real_comp = y_real;
    x_simu_comp = x_simu;
    y_simu_comp = y_simu;
end

% Thiết lập đơn vị là mét
unit = 'm';

% Tính toán sai lệch giữa quỹ đạo thực tế và mô phỏng (trên phần đã cắt)
delta_x = x_simu_comp - x_real_comp;
delta_y = y_simu_comp - y_real_comp;
errors = sqrt(delta_x.^2 + delta_y.^2);  % Khoảng cách sai lệch

% Tính toán độ sai lệch trung bình và độ lệch chuẩn
mean_error = mean(errors);  % Độ sai lệch trung bình
std_error = std(errors);    % Độ lệch chuẩn của sai lệch
max_error = max(errors);    % Sai lệch lớn nhất
min_error = min(errors);    % Sai lệch nhỏ nhất
rmse = sqrt(mean(errors.^2)); % RMSE (Root Mean Square Error)

% Hiển thị kết quả
fprintf('\n===== THỐNG KÊ SAI LỆCH QUỸ ĐẠO =====\n');
fprintf('- Sai số trung bình: %.4f %s\n', mean_error, unit);
fprintf('- Độ lệch chuẩn: %.4f %s\n', std_error, unit);
fprintf('- RMSE: %.4f %s\n', rmse, unit);
fprintf('- Sai lệch lớn nhất: %.4f %s\n', max_error, unit);
fprintf('- Sai lệch nhỏ nhất: %.4f %s\n', min_error, unit);

% Tính tổng độ dài quỹ đạo trên dữ liệu gốc
real_path_length = 0;
simu_path_length = 0;

for i = 2:length(x_real_full)
    real_path_length = real_path_length + sqrt((x_real_full(i)-x_real_full(i-1))^2 + (y_real_full(i)-y_real_full(i-1))^2);
end

for i = 2:length(x_simu_full)
    simu_path_length = simu_path_length + sqrt((x_simu_full(i)-x_simu_full(i-1))^2 + (y_simu_full(i)-y_simu_full(i-1))^2);
end

fprintf('\n===== THỐNG KÊ CHIỀU DÀI QUỸ ĐẠO =====\n');
fprintf('- Độ dài quỹ đạo thực tế: %.4f %s\n', real_path_length, unit);
fprintf('- Độ dài quỹ đạo mô phỏng: %.4f %s\n', simu_path_length, unit);
fprintf('- Khác biệt độ dài: %.4f %s (%.2f%%)\n', abs(real_path_length - simu_path_length), unit, ...
       100 * abs(real_path_length - simu_path_length) / real_path_length);

% Vẽ biểu đồ
figure('Name', 'So sánh quỹ đạo', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

% Subplot 1: Kết hợp quỹ đạo đầy đủ và vectors sai lệch
subplot(3,1,1);
hold on;

% Vẽ quỹ đạo thực tế đầy đủ
plot(x_real_full, y_real_full, '-b', 'DisplayName', 'Quỹ đạo thực tế', 'LineWidth', 2);

% Vẽ quỹ đạo mô phỏng đầy đủ
plot(x_simu_full, y_simu_full, '-r', 'DisplayName', 'Quỹ đạo mô phỏng', 'LineWidth', 2);

% Đánh dấu điểm bắt đầu và kết thúc
plot(x_real_full(1), y_real_full(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Bắt đầu thực tế');
plot(x_real_full(end), y_real_full(end), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Kết thúc thực tế');
plot(x_simu_full(1), y_simu_full(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Bắt đầu mô phỏng');
plot(x_simu_full(end), y_simu_full(end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Kết thúc mô phỏng');

% Vẽ các vector sai lệch (chỉ một số điểm để không quá lộn xộn)
sample_rate = max(1, round(length(x_real_comp) / 20));
quiver(x_real_comp(1:sample_rate:end), y_real_comp(1:sample_rate:end), ...
       delta_x(1:sample_rate:end), delta_y(1:sample_rate:end), 0.5, 'g', 'LineWidth', 1, 'DisplayName', 'Vector sai lệch');

% Thêm nhãn và chú thích
xlabel(['Tọa độ X (', unit, ')']);
ylabel(['Tọa độ Y (', unit, ')']);
title_str = sprintf('So sánh quỹ đạo: Thực tế (10.6 Hz) và Mô phỏng (33.2 Hz) - RMSE = %.4f %s', ...
     rmse, unit);
title(title_str);
legend('Location', 'best');
grid on;
axis equal;  % Đảm bảo tỷ lệ trục X và Y giống nhau

% Subplot 2: Sai lệch theo thời gian
subplot(3,1,2);
% Tạo trục thời gian cho dữ liệu đã được nội suy/cắt ngắn
t_comp = linspace(0, real_total_time, length(errors));
plot(t_comp, errors, '-b', 'LineWidth', 1.5, 'DisplayName', 'Sai lệch theo thời gian');
hold on;
plot([0, t_comp(end)], [mean_error, mean_error], '--r', 'LineWidth', 1.5, 'DisplayName', 'Sai lệch trung bình');
plot([0, t_comp(end)], [rmse, rmse], '-.g', 'LineWidth', 1.5, 'DisplayName', 'RMSE');
xlabel('Thời gian (giây)');
ylabel(['Sai lệch (', unit, ')']);
title(sprintf('Sai lệch theo thời gian: TB = %.4f %s, RMSE = %.4f %s', mean_error, unit, rmse, unit));
grid on;
legend('Location', 'best');

% Subplot 3: Histogram sai lệch
subplot(3,1,3);
histogram(errors, min(30, round(length(errors)/10)), 'Normalization', 'probability', 'FaceColor', 'b');
hold on;
xline(mean_error, '--r', 'LineWidth', 2, 'DisplayName', 'Sai lệch TB');
xline(rmse, '-.g', 'LineWidth', 2, 'DisplayName', 'RMSE');
xlabel(['Sai lệch (', unit, ')']);
ylabel('Tần suất');
title('Phân bố sai lệch');
grid on;
legend('Location', 'best');

% Chỉnh sửa layout
set(gcf, 'Color', 'w');
sgtitle(sprintf('So sánh quỹ đạo: %s vs %s', real_file, simu_file), 'FontSize', 14, 'FontWeight', 'bold');

% Lưu kết quả vào file nếu người dùng muốn
save_choice = questdlg('Bạn có muốn lưu kết quả phân tích không?', 'Lưu kết quả', 'Có', 'Không', 'Có');
if strcmp(save_choice, 'Có')
    % Tên file mặc định dựa trên tên file gốc
    [~, real_name] = fileparts(real_file);
    [~, simu_name] = fileparts(simu_file);
    default_name = sprintf('analysis_%s_vs_%s_%s.png', real_name, simu_name, datestr(now, 'yyyymmdd_HHMMSS'));
    
    [filename, pathname] = uiputfile({'*.png', 'PNG Image (*.png)'; '*.fig', 'MATLAB Figure (*.fig)'}, 'Lưu kết quả phân tích', default_name);
    if ~isequal(filename, 0)
        fullpath = fullfile(pathname, filename);
        [~, ~, ext] = fileparts(filename);
        
        if strcmp(ext, '.fig')
            savefig(gcf, fullpath);
        else
            print(gcf, fullpath, '-dpng', '-r300');
        end
        fprintf('Đã lưu kết quả phân tích vào file: %s\n', fullpath);
    end
end