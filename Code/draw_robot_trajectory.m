function animate_robot_trajectory()
    % Prompt user to select a scenario initialization file
    [filename, pathname] = uigetfile('*.json', 'Chọn file kịch bản khởi tạo');
    if isequal(filename, 0)
        % User canceled - exit the function
        disp('Đã hủy việc chọn file kịch bản.');
        return;
    end
    
    % Load the scenario file
    fullFilePath = fullfile(pathname, filename);
    try
        scenarioData = jsondecode(fileread(fullFilePath));
        disp('Đã đọc file kịch bản thành công.');
        
        % Make sure file contains required data
        if ~isfield(scenarioData, 'robots') || ~isfield(scenarioData, 'targets')
            error('File kịch bản không đúng định dạng. File phải chứa các trường "robots" và "targets".');
        end
    catch ME
        errordlg(['Lỗi khi đọc file kịch bản: ' ME.message], 'Lỗi đọc file');
        return;
    end
    
    % Tạo figure mới
    figure('Name', 'Robot di chuyển theo quỹ đạo', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
    
    % Thiết lập trục tọa độ với trục y đảo ngược
    ax = axes('Color', 'w');
    set(ax, 'YDir', 'reverse'); % ĐẢO NGƯỢC TRỤC Y
    
    % Thiết lập giới hạn trục và tỷ lệ
    xlim([-1 6]);
    ylim([-2 4]);
    axis equal;
    
    % Vẽ khung viền
    rectangle('Position', [-1, -2, 7, 6], 'EdgeColor', 'k', 'LineWidth', 2);
    
    % Định nghĩa thông số robot
    ROBOT_RADIUS = 0.24 / 2;
    AXLE_LENGTH = 0.175;
    WHEEL_WIDTH = 0.01;
    WHEEL_LENGTH = 0.04;
    robot_color = [0.6 0.8 1]; % Màu xanh nhạt
    
    % Vẽ các điểm đích từ dữ liệu kịch bản
    hold on;
    target_points = cell(length(scenarioData.targets), 1);
    for i = 1:length(scenarioData.targets)
        target_x = scenarioData.targets(i).x;
        target_y = scenarioData.targets(i).y;
        target_points{i} = plot(target_x, target_y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    end
    
    % Vẽ robot ở vị trí khởi tạo
    robots_handles = struct('body', {}, 'left_wheel', {}, 'right_wheel', {}, 'heading_arrow', {});
    
    for i = 1:length(scenarioData.robots)
        % Lấy dữ liệu robot từ file kịch bản
        robot_id = scenarioData.robots(i).id;
        x_initial = scenarioData.robots(i).x;
        y_initial = scenarioData.robots(i).y;
        initial_theta_degrees = scenarioData.robots(i).theta;
        initial_theta = deg2rad(initial_theta_degrees); % Chuyển đổi từ độ sang radian
        
        % Hiển thị thông tin robot
        disp(['Robot ', num2str(i), ':']);
        disp(['  ID: ', num2str(robot_id)]);
        disp(['  Vị trí khởi tạo: (', num2str(x_initial), ', ', num2str(y_initial), ')']);
        disp(['  Góc khởi tạo: ', num2str(initial_theta_degrees), ' độ (', num2str(initial_theta), ' rad)']);
        
        % Ma trận xoay
        Rmat = [cos(initial_theta), -sin(initial_theta); 
                sin(initial_theta), cos(initial_theta)];
        
        % Vẽ thân tròn của robot
        theta_vals = linspace(0, 2*pi, 30);
        circ_x = ROBOT_RADIUS * cos(theta_vals);
        circ_y = ROBOT_RADIUS * sin(theta_vals);
        circle = Rmat * [circ_x; circ_y];
        robots_handles(i).body = fill(circle(1,:) + x_initial, circle(2,:) + y_initial, robot_color, 'EdgeColor', 'k', 'LineWidth', 1);
        
        % Vẽ bánh xe trái
        wheel_shape = [-WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                        WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                        WHEEL_LENGTH/2,  WHEEL_WIDTH/2;
                       -WHEEL_LENGTH/2,  WHEEL_WIDTH/2]';
        wheel_left = Rmat * wheel_shape + [x_initial - sin(initial_theta)*(AXLE_LENGTH/2); 
                                   y_initial + cos(initial_theta)*(AXLE_LENGTH/2)];
        robots_handles(i).left_wheel = fill(wheel_left(1,:), wheel_left(2,:), 'k');
        
        % Vẽ bánh xe phải
        wheel_right = Rmat * wheel_shape + [x_initial + sin(initial_theta)*(AXLE_LENGTH/2); 
                                    y_initial - cos(initial_theta)*(AXLE_LENGTH/2)];
        robots_handles(i).right_wheel = fill(wheel_right(1,:), wheel_right(2,:), 'k');
        
        % Vẽ mũi tên hướng
        robots_handles(i).heading_arrow = quiver(x_initial, y_initial, cos(initial_theta)*0.15, sin(initial_theta)*0.15, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);
    end
    
    % Thêm nút để chọn file tọa độ di chuyển
    play_button = uicontrol('Style', 'pushbutton', ...
                             'String', 'Move Robot', ...
                             'Position', [20, 20, 100, 30], ...
                             'Callback', @playRobotTrajectory);
    
    hold off;
    drawnow;
    
    % Hàm callback cho nút Move Robot
    function playRobotTrajectory(src, event)
        % Cho phép người dùng chọn file tọa độ
        [filename, pathname] = uigetfile({'*.csv'}, 'Chọn file tọa độ di chuyển');
        
        if isequal(filename, 0) || isequal(pathname, 0)
            % Người dùng đã hủy
            return;
        end
        
        % Đọc dữ liệu từ file CSV
        filePath = fullfile(pathname, filename);
        try
            data = readtable(filePath);
            if ~isempty(data)
                % Kiểm tra cấu trúc dữ liệu
                if ismember('X', data.Properties.VariableNames) && ismember('Y', data.Properties.VariableNames)
                    % Lưu tọa độ X và Y
                    x_trajectory = data.X;
                    y_trajectory = data.Y;
                    
                    % Kết hợp với thông tin robot từ file kịch bản
                    % Lấy robot đầu tiên từ scenarioData để di chuyển
                    robot_id = scenarioData.robots(1).id;
                    
                    % Hiển thị thông tin
                    msgbox(sprintf('Sẽ di chuyển robot ID: %d theo file: %s\nSố điểm: %d', ...
                           robot_id, filename, length(x_trajectory)), 'Thông tin', 'help');
                    
                    % Bắt đầu di chuyển robot
                    animateRobotPath(x_trajectory, y_trajectory, scenarioData.robots(1));
                else
                    errordlg('File không đúng định dạng. Cần có cột X và Y.', 'Lỗi định dạng');
                end
            else
                errordlg('File không chứa dữ liệu.', 'Lỗi dữ liệu');
            end
        catch ME
            errordlg(['Lỗi khi đọc file: ', ME.message], 'Lỗi');
        end
    end
    
    % Thay đổi cách hiển thị thông tin trong hàm animateRobotPath
    function animateRobotPath(x_trajectory, y_trajectory, robotData)
        % Định nghĩa thông số robot
        ROBOT_RADIUS = 0.24 / 2;
        AXLE_LENGTH = 0.175;
        WHEEL_WIDTH = 0.01;
        WHEEL_LENGTH = 0.04;
        robot_color = [0.6 0.8 1]; % Màu xanh nhạt
        
        % Tính toán góc hướng (theta) từ quỹ đạo
        theta = zeros(size(x_trajectory));
        
        % Sử dụng cửa sổ trượt để làm mềm góc
        window_size = 5;
        look_ahead_index = min(window_size, length(x_trajectory));
        
        % Tính góc hướng ban đầu dựa vào điểm đủ xa đó
        dx_initial = x_trajectory(look_ahead_index) - x_trajectory(1);
        dy_initial = y_trajectory(look_ahead_index) - y_trajectory(1);
        theta(1) = atan2(dy_initial, dx_initial);
        
        % Áp dụng hướng ban đầu cho các điểm đầu tiên để tránh xoay đột ngột
        for i = 2:min(window_size, length(theta))
            theta(i) = theta(1);
        end
        
        % Tính góc cho các điểm còn lại
        for i = window_size+1:length(x_trajectory)
            dx = x_trajectory(i) - x_trajectory(i-1);
            dy = y_trajectory(i) - y_trajectory(i-1);
            if dx ~= 0 || dy ~= 0 % Tránh chia cho 0
                raw_theta = atan2(dy, dx);
                
                % Áp dụng smoothing để tránh xoay quá đột ngột
                theta(i) = 0.8 * raw_theta + 0.2 * theta(i-1);
            else
                theta(i) = theta(i-1); % Giữ nguyên góc trước đó
            end
        end
        
        % Hiển thị thông tin - Thêm kiểm tra figure còn tồn tại
        fig_handle = gcf;
        if isvalid(fig_handle)
            info_text = text(5, -1.5, 'Điểm: 0/0 | Nhấn phím bất kỳ để bắt đầu...', 'FontSize', 12);
            drawnow;
            
            % Thêm try-catch cho phần đợi phím bấm
            try
                pause;
            catch ME
                return; % Thoát nếu figure bị đóng trong quá trình đợi
            end
        else
            return; % Thoát nếu figure không còn hợp lệ
        end
        
        % Kiểm tra info_text còn hợp lệ không trước khi sử dụng
        if ~exist('info_text', 'var') || ~isvalid(info_text)
            % Nếu không còn hợp lệ, tạo lại text object hoặc thoát
            return;
        end
        
        % Tạo timer để hiển thị thời gian
        startTime = tic;
        
        % Xác định tốc độ (số frame mỗi giây)
        fps = 30;
        total_frames = min(length(x_trajectory), 1000); % Giới hạn số lượng frame tối đa
        
        % Tạo các handle cho robot
        robot_body = [];
        left_wheel = [];
        right_wheel = [];
        heading_arrow = [];
        path_line = [];
        
        % Khởi tạo mảng lưu quỹ đạo đã đi
        path_history = zeros(total_frames, 2);
        
        % Vòng lặp hoạt ảnh
        for frame = 1:total_frames
            % Kiểm tra figure còn tồn tại không
            if ~isvalid(fig_handle)
                return; % Thoát nếu figure đã bị đóng
            end
            
            % Kiểm tra info_text còn hợp lệ không
            if isvalid(info_text)
                % Cập nhật thông tin hiển thị
                elapsed_time = toc(startTime);
                info_text.String = sprintf('Điểm: %d/%d | Thời gian: %.2fs', ...
                                         frame, total_frames, elapsed_time);
            end
            
            % Phần còn lại của vòng lặp với bổ sung xử lý lỗi
            try
                % Lấy vị trí hiện tại
                x = x_trajectory(frame);
                y = y_trajectory(frame);
                current_theta = theta(frame);
                
                % Lưu vị trí vào lịch sử quỹ đạo
                path_history(frame, :) = [x, y];
                
                % Xóa robot cũ
                if ~isempty(robot_body) && isvalid(robot_body)
                    delete(robot_body);
                end
                if ~isempty(left_wheel) && isvalid(left_wheel)
                    delete(left_wheel);
                end
                if ~isempty(right_wheel) && isvalid(right_wheel)
                    delete(right_wheel);
                end
                if ~isempty(heading_arrow) && isvalid(heading_arrow)
                    delete(heading_arrow);
                end
                
                % Ma trận xoay
                Rmat = [cos(current_theta), -sin(current_theta); 
                        sin(current_theta), cos(current_theta)];
                
                % Vẽ quỹ đạo đã đi qua
                if ~isempty(path_line) && isvalid(path_line)
                    delete(path_line);
                end
                hold on;
                path_line = plot(path_history(1:frame, 1), path_history(1:frame, 2), 'b-', 'LineWidth', 1);
                
                % Vẽ thân tròn của robot
                theta_vals = linspace(0, 2*pi, 30);
                circ_x = ROBOT_RADIUS * cos(theta_vals);
                circ_y = ROBOT_RADIUS * sin(theta_vals);
                circle = Rmat * [circ_x; circ_y];
                robot_body = fill(circle(1,:) + x, circle(2,:) + y, robot_color, 'EdgeColor', 'k', 'LineWidth', 1);
                
                % Vẽ bánh xe trái
                wheel_shape = [-WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                                WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                                WHEEL_LENGTH/2,  WHEEL_WIDTH/2;
                               -WHEEL_LENGTH/2,  WHEEL_WIDTH/2]';
                wheel_left = Rmat * wheel_shape + [x - sin(current_theta)*(AXLE_LENGTH/2); 
                                           y + cos(current_theta)*(AXLE_LENGTH/2)];
                left_wheel = fill(wheel_left(1,:), wheel_left(2,:), 'k');
                
                % Vẽ bánh xe phải
                wheel_right = Rmat * wheel_shape + [x + sin(current_theta)*(AXLE_LENGTH/2); 
                                            y - cos(current_theta)*(AXLE_LENGTH/2)];
                right_wheel = fill(wheel_right(1,:), wheel_right(2,:), 'k');
                
                % Vẽ mũi tên hướng
                heading_arrow = quiver(x, y, cos(current_theta)*0.15, sin(current_theta)*0.15, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);
                
                hold off;
                
                % Đảm bảo robot luôn nằm trong tầm nhìn
                xlim([-1 6]);
                ylim([-2 4]);
                
                % Cập nhật hiển thị và tạm dừng để tạo hoạt ảnh mượt mà
                drawnow;
                pause(1/fps);
            catch ME
                % Ghi log lỗi nhưng không làm gián đoạn chương trình
                disp(['Lỗi trong vòng lặp hoạt ảnh: ', ME.message]);
                continue;
            end
        end
        
        % Thêm try-catch cho phần kết thúc
        try
            % Cập nhật thông tin khi hoàn thành
            if exist('info_text', 'var') && isvalid(info_text)
                info_text.String = sprintf('Hoàn thành! %d điểm | Thời gian: %.2fs', total_frames, toc(startTime));
            end
            
            % Hiển thị thông báo hoàn thành
            msgbox(sprintf('Đã di chuyển robot theo quỹ đạo!\nThời gian: %.2f giây', toc(startTime)), 'Hoàn thành');
        catch ME
            disp(['Lỗi khi hoàn thành hoạt ảnh: ', ME.message]);
        end
    end
end