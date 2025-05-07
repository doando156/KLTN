function varargout = simu_gui(varargin)
% SIMU_GUI MATLAB code for simu_gui.fig
%      SIMU_GUI, by itself, creates a new SIMU_GUI or raises the existing
%      singleton*.
%
%      H = SIMU_GUI returns the handle to a new SIMU_GUI or the handle to
%      the existing singleton*.
%
%      SIMU_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMU_GUI.M with the given input arguments.
%
%      SIMU_GUI('Property','Value',...) creates a new SIMU_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before simu_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to simu_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help simu_gui

% Last Modified by GUIDE v2.5 07-May-2025 17:25:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @simu_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @simu_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before simu_gui is made visible.
function simu_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to simu_gui (see VARARGIN)

% Choose default command line output for simu_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = simu_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function pushbutton1_Callback(hObject, eventdata, handles)
    % Cài đặt không gian mô phỏng
    cla;
    set(gca, 'Color', 'w');
    xlim([-5 5]);         % Chiều ngang 8 mét
    ylim([-3 3]);         % Chiều cao 8 mét
    axis equal;
    rectangle('Position', [-5, -3, 10, 6], 'EdgeColor', 'k', 'LineWidth', 2);

    % Kiểm tra chế độ hiện tại từ popupmenu
    contents = cellstr(get(handles.popupmenu1, 'String'));  % Lấy danh sách các lựa chọn trong menu
    selectedOption = contents{get(handles.popupmenu1, 'Value')};  % Lấy giá trị đã chọn
    
    % Kiểm tra chế độ hiện tại
    if strcmp(selectedOption, 'Simulation')
        % Chế độ mô phỏng: Đọc dữ liệu từ file current_position.json (chỉnh tay)
        robotData = jsondecode(fileread('current_position.json'));
    else
        % Chế độ thực tế: Đọc dữ liệu từ file data.json (cập nhật từ camera)
        DATA_FILE_PATH = 'C:/Users/Do Doan/OneDrive/Documents/HocTapUet/DATN_Final/aruco_python/data.json';
        if isfile(DATA_FILE_PATH)
            % Đọc dữ liệu từ file data.json
            cameraData = jsondecode(fileread(DATA_FILE_PATH));
            
            % Chuyển đổi dữ liệu camera thành định dạng tương ứng cho mô phỏng
            robotData = [];
            ids = fieldnames(cameraData); % Lấy các id robot từ dữ liệu JSON
            for i = 1:length(ids)
                disp(">> ID lấy từ camera:");
                disp(ids{i});
                % Chuẩn hóa ID chỉ lấy phần số
                clean_id = regexp(ids{i}, '\d+', 'match');
                if isempty(clean_id)
                    warning("❌ Không tìm thấy ID số trong %s", ids{i});
                    continue;
                end
                robot_id = str2double(clean_id{1});               
                disp(">> ID sau khi loại chữ lấy từ camera:");
                disp(robot_id);
                robots(i).id = robot_id;
                robots(i).x = cameraData.(ids{i}).x / 100;
                robots(i).y = cameraData.(ids{i}).y / 100;
                robots(i).theta = cameraData.(ids{i}).theta * pi / 180;
            end
            
            %  Move these lines INSIDE the nested if
            robotData = robots;
            disp(">> Đã nạp số robot từ camera:");
            disp(length(robotData));  % Phải > 0
        else
            error('File data.json không tồn tại tại đường dẫn: %s', DATA_FILE_PATH);
        end
    end
    targetData = jsondecode(fileread('target_position.json'));

    hold on;

    % Vẽ các điểm target
    for i = 1:length(targetData)
        plot(targetData(i).x, targetData(i).y, 'rx', 'MarkerSize', 3, 'LineWidth', 1);
    end

    % Thông số robot thực tế
    ROBOT_RADIUS = 0.24 / 2;
    AXLE_LENGTH = 0.175;
    WHEEL_WIDTH = 0.01;
    WHEEL_LENGTH = 0.04;
    WHEEL_RADIUS = 0.0325;

    % Lưu các handle robot và mũi tên
    handles.robotsPlot = gobjects(length(robotData), 1); 
    handles.headingArrows = gobjects(length(robotData), 1); 
    handles.leftWheels = gobjects(length(robotData), 1);
    handles.rightWheels = gobjects(length(robotData), 1);

   % Vẽ từng robot
    for i = 1:length(robotData)
        x = robotData(i).x;
        y = robotData(i).y;
        theta = robotData(i).theta * pi / 180;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

        % Vẽ thân tròn
        theta_vals = linspace(0, 2*pi, 100);
        circ_x = ROBOT_RADIUS * cos(theta_vals);
        circ_y = ROBOT_RADIUS * sin(theta_vals);
        circle = R * [circ_x; circ_y];
        handles.robotsPlot(i) = fill(circle(1,:) + x, circle(2,:) + y, [0.6 0.8 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
        

        % Vẽ bánh trái
        wheel_shape = [-WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                        WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                        WHEEL_LENGTH/2,  WHEEL_WIDTH/2;
                       -WHEEL_LENGTH/2,  WHEEL_WIDTH/2]';
        wheel_left = R * wheel_shape + [x - sin(theta)*(AXLE_LENGTH/2); y + cos(theta)*(AXLE_LENGTH/2)];
        handles.leftWheels(i) = fill(wheel_left(1,:), wheel_left(2,:), 'k');

        % Vẽ bánh phải
        wheel_right = R * wheel_shape + [x + sin(theta)*(AXLE_LENGTH/2); y - cos(theta)*(AXLE_LENGTH/2)];
        handles.rightWheels(i) = fill(wheel_right(1,:), wheel_right(2,:), 'k');

        % Vẽ mũi tên heading
        handles.headingArrows(i) = quiver(x, y, cos(theta)*0.15, sin(theta)*0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
        % Vẽ mũi tên heading
        %handles.headingArrows(i) = quiver(x, y, cos(theta), sin(theta), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);

    end

    hold off;
    % Lưu lại handles
    guidata(hObject, handles);

    msgbox('Khởi tạo không gian mô phỏng thành công!', 'Thông báo');


global stopFlag;  % Khai báo biến toàn cục stopFlag

% Khởi tạo stopFlag = false (robot chạy)
stopFlag = false;

function pushbutton2_Callback(hObject, eventdata, handles)
    % Keep the old parameters, global variables
    global cameraData;
    global ids;
    global s;
    global stopFlag;  % Khai báo sử dụng biến toàn cục stopFlag
    
    % Kiểm tra nếu stopFlag là true, thoát vòng lặp ngay lập tức
    if stopFlag
        disp('Mô phỏng đã bị dừng');
        return; % Thoát khỏi vòng lặp
    end


    
    % Constants and parameters
    DATA_FILE_PATH = 'C:/Users/Do Doan/OneDrive/Documents/HocTapUet/DATN_Final/aruco_python/data.json';
    threshold = 0.5;
    dt = 0.1;
    kt = 1;
    kd = 0.2;
    ROBOT_RADIUS = 0.24 / 2;
    AXLE_LENGTH = 0.175;
    WHEEL_RADIUS = 0.0325;
    WHEEL_WIDTH = 0.01;
    WHEEL_LENGTH = 0.04;
    
    % Check the current mode from the popup menu
    contents = cellstr(get(handles.popupmenu1, 'String'));
    selectedOption = contents{get(handles.popupmenu1, 'Value')};
    
    % Read the robot data based on the selected option
    if strcmp(selectedOption, 'Simulation')
        % Load data from simulation
        if isfile('current_position.json')
            robotData = jsondecode(fileread('current_position.json'));
        else
            error('Simulation data file current_position.json not found.');
        end
    else
        % Real-World: Read data from the camera (data.json)
        try
            s = serialport("COM29", 115200, "Timeout", 1);
        catch
            error('Cannot open COM29 port. Please check the Bluetooth HC-05 connection.');
        end
        
        if isfile(DATA_FILE_PATH)
            % Read data from the camera file (data.json)
            cameraData = jsondecode(fileread(DATA_FILE_PATH));
            robots = struct('id', {}, 'x', {}, 'y', {}, 'theta', {}, 'targetIndex', {});
            ids = fieldnames(cameraData);  % Get the robot IDs from camera data
            
            % Convert camera data to appropriate format for simulation
            for i = 1:length(ids)
                clean_id = regexp(ids{i}, '\d+', 'match');
                if isempty(clean_id)
                    warning("❌ No numeric ID found in %s", ids{i});
                    continue;
                end
                robot_id = str2double(clean_id{1});
                robots(i).id = robot_id;
                robots(i).x = cameraData.(ids{i}).x / 100;  % Convert to meters
                robots(i).y = cameraData.(ids{i}).y / 100;  % Convert to meters
                robots(i).theta = cameraData.(ids{i}).theta * pi /180;
            end
            
            robotData = robots;  % Assign converted robot data to the robotData variable
            disp(">> Loaded robot data from the camera:");
            disp(length(robotData));  % Must be > 0
        else
            error('File data.json not found at path: %s', DATA_FILE_PATH);
        end
    end
    
    % Load the target positions from the target_position.json file
    targetData = jsondecode(fileread('target_position.json'));
    target_x = targetData(1).x;
    target_y = targetData(1).y;

    % Step 1: Calculate the distance from each robot to the first target
    distances = zeros(length(robotData), 1);
    for i = 1:length(robotData)
        dx = robotData(i).x - target_x;
        dy = robotData(i).y - target_y;
        distances(i) = sqrt(dx^2 + dy^2);
    end
    
    % Step 2: Sort the robots by their distance to the target (leader will be the closest)
    [~, sorted_indices] = sort(distances);
    robotData = robotData(sorted_indices);  % The robot closest to the target is now the leader
    
    % Step 3: Assign robots to the leader-follower structure based on their sorted distances
    leader = robotData(1);  % The robot closest to the target is the leader
    followers = robotData(2:end);  % Remaining robots are followers
    % Initialize follower_order to avoid errors
    follower_order = followers;  % Initially, set the follower order to just followers

    
    % Now we organize the followers based on the distance to the leader and then to the previous follower
    for i = 1:length(followers)
        if i == 1
            % The first follower is the one closest to the leader
            distances_to_leader = zeros(length(followers), 1);
            for j = 1:length(followers)
                dx = followers(j).x - leader.x;
                dy = followers(j).y - leader.y;
                distances_to_leader(j) = sqrt(dx^2 + dy^2);
            end
            [~, sorted_follower_indices] = sort(distances_to_leader);
            follower_order = followers(sorted_follower_indices);  % Sorted followers by distance to leader
        else
            % For subsequent followers, sort by distance to the previous follower
            previous_follower = follower_order(i-1);
            distances_to_previous_follower = zeros(length(followers), 1);
            for j = 1:length(followers)
                dx = followers(j).x - previous_follower.x;
                dy = followers(j).y - previous_follower.y;
                distances_to_previous_follower(j) = sqrt(dx^2 + dy^2);
            end
            [~, sorted_follower_indices] = sort(distances_to_previous_follower);
            follower_order = followers(sorted_follower_indices);  % Sorted followers by distance to previous follower
        end
    end

    % Now, the robotData array contains the leader first, followed by followers in order.
    robotData = [leader; follower_order];  % Complete robotData with leader and sorted followers
    
    % Step 1: Clear the old robot components (body, wheels, arrows)
    if isfield(handles, 'robotsPlot')
        for i = 1:length(handles.robotsPlot)
            if ishandle(handles.robotsPlot(i))
                delete(handles.robotsPlot(i));
            end
        end
    end

    if isfield(handles, 'headingArrows')
        for i = 1:length(handles.headingArrows)
            if ishandle(handles.headingArrows(i))
                delete(handles.headingArrows(i));
            end
        end
    end

    if isfield(handles, 'leftWheels')
        for i = 1:length(handles.leftWheels)
            if ishandle(handles.leftWheels(i))
                delete(handles.leftWheels(i));
            end
        end
    end

    if isfield(handles, 'rightWheels')
        for i = 1:length(handles.rightWheels)
            if ishandle(handles.rightWheels(i))
                delete(handles.rightWheels(i));
            end
        end
    end

    % Khởi tạo robot
    handles.robotsPlot = gobjects(length(robotData), 1);
    handles.headingArrows = gobjects(length(robotData), 1);
    handles.leftWheels = gobjects(length(robotData), 1);
    handles.rightWheels = gobjects(length(robotData), 1);

    for i = 1:length(robotData)
        % Lưu thông tin robot vào struct array robots
        robots(i).id = robotData(i).id;        % Sử dụng robots(i) thay vì robots{i}
        robots(i).x = robotData(i).x;
        robots(i).y = robotData(i).y;
        robots(i).theta = robotData(i).theta;
        robots(i).targetIndex = 1;  % Gán chỉ số mục tiêu ban đầu cho mỗi robot
    end
    if strcmp(selectedOption, 'Simulation')
    % Tạo file JSON nếu chưa có
        if ~isfile('current_position_robot_simu.json')
            robotPositions = struct('id', {}, 'x', {}, 'y', {}, 'theta', {});
            for i = 1:length(robotData)
                robotPositions(i).id = robotData(i).id;
                robotPositions(i).x = robotData(i).x;
                robotPositions(i).y = robotData(i).y;
                robotPositions(i).theta = robotData(i).theta;
            end
            fid = fopen('current_position_robot_simu.json', 'w');
            if fid == -1, error('Không thể mở file để ghi!'); end
            fprintf(fid, '%s', jsonencode(robotPositions));
            fclose(fid);
        end
    end
    
    % Mô phỏng
    for step = 1:2000
        % Kiểm tra nếu stopFlag là true, thoát vòng lặp ngay lập tức
        if stopFlag
            disp('Mô phỏng đã bị dừng');
            return; % Thoát khỏi vòng lặp
        end
        disp(['Giá trị của stopFlag: ', num2str(stopFlag)]);  % In giá trị của stopFlag
        
        if strcmp(selectedOption, 'Simulation')
         % Khởi tạo struct chỉ khi cần thiết trong chế độ Simulation
         robotPositions = struct('id', {}, 'x', {}, 'y', {}, 'theta', {});
         for i = 1:length(robots)
            
            robot = robots(i);  % Sử dụng parentheses indexing thay vì brace indexing
            fprintf('robot.x: %.2f\n', robot.x);
            fprintf('robot.y: %.2f\n', robot.y);
            % Tính khoảng cách đến mục tiêu
            if i == 1
                target = targetData(robot.targetIndex);
                dx = target.x - robot.x;        
                dy = target.y - robot.y;
            else
                target = robots(i-1);
                dx = target.x - robot.x;
                dy = target.y - robot.y;
                distance_to_leader = sqrt(dx^2 + dy^2);
                if distance_to_leader < threshold
                    robots(i) = robot;
                    continue;
                end
            end
            % Tính khoảng cách và góc hướng tới mục tiêu
            distance = sqrt(dx^2 + dy^2);
            fprintf('distance: %.2f\n', distance);
            angle_to_target = atan2(dy, dx);
            fprintf('angle_to_target: %.2f\n', angle_to_target);
            angle_diff = wrapToPi(angle_to_target - robot.theta);
            fprintf('angle_diff: %.2f\n', angle_diff);
            % Chuyển sang target tiếp theo nếu robot leader đã đến gần target
            if i == 1 && distance < 0.2
                % Kiểm tra follower đã bám sát chưa
                all_followers_ready = true;
                for j = 2:length(robots)
                    dx_f = robots(j-1).x - robots(j).x;
                    dy_f = robots(j-1).y - robots(j).y;
                    d_follower = sqrt(dx_f^2 + dy_f^2);
                    if d_follower > threshold
                        all_followers_ready = false;
                        break;
                    end
                end

                if all_followers_ready
                    if robot.targetIndex < length(targetData)
                        robot.targetIndex = robot.targetIndex + 1;
                    else
                        % Nếu đã đến target cuối và follower đã bám sát: kết thúc mô phỏng
                        msgbox(' Mô phỏng kết thúc: Tất cả robot đã đến đích và bám sát!', 'Hoàn thành');
                        return;
                    end
                else
                    % Nếu follower chưa đến, leader đứng yên
                    robots(i) = robot;
                    continue;
                end
            end
            % Tính vận tốc và tốc độ góc (omega)
            v_simu = min(0.01, distance);
            w_simu = angle_diff * 0.03;


            % Áp dụng công thức tính vận tốc bánh xe trong (mô phỏng)
            v_left_simu = (v_simu - w_simu * (AXLE_LENGTH / 2)) / WHEEL_RADIUS;  % Tính vận tốc bánh trái
            v_right_simu = (v_simu + w_simu * (AXLE_LENGTH / 2)) / WHEEL_RADIUS; % Tính vận tốc bánh phải


            % Tính trung bình vận tốc robot
            v_robot_simu = (v_left_simu + v_right_simu) / 2;
            w_robot_simu = (v_right_simu - v_left_simu) / AXLE_LENGTH;

            % Cập nhật vị trí robot trong mô phỏng
            robot.x = robot.x + v_robot_simu * cos(robot.theta) * dt;
            robot.y = robot.y + v_robot_simu * sin(robot.theta) * dt;
            robot.theta = wrapToPi(robot.theta + w_robot_simu * dt);
            robots(i) = robot;

            robotPositions(i).id = robots(i).id;
            robotPositions(i).x = robots(i).x;
            robotPositions(i).y = robots(i).y;
            robotPositions(i).theta = robots(i).theta;

            fid = fopen('current_position_robot_simu.json', 'w');
            if fid == -1, error('Không thể mở file để ghi!'); end
            fprintf(fid, '%s', jsonencode(robotPositions));
            fclose(fid);
         end  
        else
            disp('Chế độ: Real-World');
            for i = 1:length(robots)            
                robot = robots(i);  % Sử dụng parentheses indexing thay vì brace indexing
                fprintf('robot.x: %.2f\n', robot.x);
                fprintf('robot.y: %.2f\n', robot.y);
                fprintf('robot.theta: %.2f\n', robot.theta);
                % Tính khoảng cách đến mục tiêu
                if i == 1
                    target = targetData(robot.targetIndex);
                    dx = target.x - robot.x;        
                    dy = target.y - robot.y;
                else
                    target = robots(i-1);
                    dx = target.x - robot.x;
                    dy = target.y - robot.y;
                    distance_to_leader = sqrt(dx^2 + dy^2);
                    if distance_to_leader < threshold
                        cmd_str = sprintf('%d,0,0;', robot.id);
                        send_all_pwm_commands(cmd_str);  % Gửi lệnh dừng robot thực tế
                        % Đọc lại dữ liệu vị trí từ file camera sau khi robot dừng lại
                        if isfile(DATA_FILE_PATH)
                            try
                                cameraData = jsondecode(fileread(DATA_FILE_PATH));
                                disp(cameraData);  % Kiểm tra xem cấu trúc của cameraData có đúng như bạn mong đợi không
                                % Tìm lại đúng ID của robot hiện tại
                                id_str = ['x', num2str(robot.id)];
                                if isfield(cameraData, id_str)
                                    robots(i).x = cameraData.(id_str).x / 100;  % Cập nhật vị trí từ camera (chuyển đổi từ cm sang m)
                                    robots(i).y = cameraData.(id_str).y / 100;  % Cập nhật vị trí từ camera (chuyển đổi từ cm sang m)
                                    robots(i).theta = cameraData.(id_str).theta * pi / 180;  % Cập nhật góc hướng
                                else
                                    warning("Không tìm thấy ID %s trong data.json", id_str);
                                end
                            catch ME
                                warning("Lỗi khi đọc file data.json: %s", ME.message);
                            end
                        end
                        continue;
                    end
                end
                % Tính khoảng cách và góc hướng tới mục tiêu
                distance = sqrt(dx^2 + dy^2);
                fprintf('distance: %.2f\n', distance);
                fprintf('robot.x_after_distance_caculate: %.2f\n', robot.x);
                fprintf('robot.y_after_distance_caculate: %.2f\n', robot.y);
                angle_to_target = atan2(dy, dx);
                fprintf('angle_to_target: %.2f\n', angle_to_target);
                angle_diff = wrapToPi(angle_to_target - robot.theta);
                fprintf('angle_diff: %.2f\n', angle_diff);
                % Chuyển sang target tiếp theo nếu robot leader đã đến gần target
                if i == 1 && distance < 0.2
                    % Kiểm tra follower đã bám sát chưa
                    all_followers_ready = true;
                    for j = 2:length(robots)
                        dx_f = robots(j-1).x - robots(j).x;
                        dy_f = robots(j-1).y - robots(j).y;
                        d_follower = sqrt(dx_f^2 + dy_f^2);
                        if d_follower > threshold
                            all_followers_ready = false;
                            break;
                        end
                    end

                    if all_followers_ready
                        if robot.targetIndex < length(targetData)
                            robot.targetIndex = robot.targetIndex + 1;
                        else
                            cmd_str = sprintf('%d,0,0;', robot.id);
                            send_all_pwm_commands(cmd_str);  % Gửi lệnh dừng robot thực tế
                            % Nếu đã đến target cuối và follower đã bám sát: kết thúc mô phỏng
                            msgbox(' Mô phỏng kết thúc: Tất cả robot đã đến đích và bám sát!', 'Hoàn thành');
                            return;
                        end
                    else
                        % Nếu follower chưa đến, leader đứng yên
                        cmd_str = sprintf('%d,0,0;', robot.id);
                        send_all_pwm_commands(cmd_str);  % Gửi lệnh dừng robot thực tế
                        % Đọc lại dữ liệu vị trí từ file camera sau khi robot dừng lại
                        if isfile(DATA_FILE_PATH)
                            try
                                cameraData = jsondecode(fileread(DATA_FILE_PATH));
                                disp(cameraData);  % Kiểm tra xem cấu trúc của cameraData có đúng như bạn mong đợi không
                                % Tìm lại đúng ID của robot hiện tại
                                id_str = ['x', num2str(robot.id)];
                                if isfield(cameraData, id_str)
                                    robots(i).x = cameraData.(id_str).x / 100;  % Cập nhật vị trí từ camera (chuyển đổi từ cm sang m)
                                    robots(i).y = cameraData.(id_str).y / 100;  % Cập nhật vị trí từ camera (chuyển đổi từ cm sang m)
                                    robots(i).theta = cameraData.(id_str).theta * pi / 180 ;  % Cập nhật góc hướng
                                else
                                    warning("Không tìm thấy ID %s trong data.json", id_str);
                                end
                            catch ME
                                warning("Lỗi khi đọc file data.json: %s", ME.message);
                            end
                        end
                        continue;
                    end
                end
                % Tính vận tốc tuyến tính và góc quay thực tế
                v_real = kd * distance;
                w_real = -kt * angle_diff;
                % In ra giá trị của v_real và w_real
                fprintf('v_real (velocity): %.2f\n', v_real);
                fprintf('w_real (angular velocity): %.2f\n', w_real);


                % Tính vận tốc góc 2 bánh (rad/s)
                v_left_real = (v_real - w_real * (AXLE_LENGTH / 2)) / WHEEL_RADIUS;
                v_right_real = (v_real + w_real * (AXLE_LENGTH / 2)) / WHEEL_RADIUS;
                % In ra giá trị của v_left_real và v_right_real
                fprintf('v_left_real: %.2f\n', v_left_real);
                fprintf('v_right_real: %.2f\n', v_right_real);

                % Chuyển sang PWM
                [pwm_left, pwm_right] = map_velocity(v_left_real, v_right_real, 50);
                
                % In ra giá trị PWM
                fprintf('pwm_left: %.2f\n', pwm_left);
                fprintf('pwm_right: %.2f\n', pwm_right);
                
                
                % Tạo chuỗi theo định dạng id,pwmL,pwmR;
                cmd_str = sprintf('%d,%.2f,%.2f;', robot.id, pwm_left, pwm_right);
                
                % Gửi ngay lập tức gói tin cho robot
                send_all_pwm_commands(cmd_str);
                
                
                % Đọc lại dữ liệu vị trí mới nhất từ file
                if isfile(DATA_FILE_PATH)
                    try
                        cameraData = jsondecode(fileread(DATA_FILE_PATH));
                        disp(cameraData);  % Kiểm tra xem cấu trúc của cameraData có đúng như bạn mong đợi không
                        % Tìm lại đúng ID của robot hiện tại
                        id_str = ['x', num2str(robot.id)];
                        if isfield(cameraData, id_str)
                            robots(i).x = cameraData.(id_str).x / 100;
                            robots(i).y = cameraData.(id_str).y / 100;
                            robots(i).theta = cameraData.(id_str).theta * pi / 180;
                        else
                            warning("Không tìm thấy ID %s trong data.json", id_str);
                        end
                    catch ME
                        warning("Lỗi khi đọc file data.json: %s", ME.message);
                    end
                end              
                
            end
        end
                       
        % Xóa robot & mũi tên & bánh xe cũ
        if isfield(handles, 'robotsPlot')
            for i = 1:length(handles.robotsPlot)
                if ishandle(handles.robotsPlot(i))
                    delete(handles.robotsPlot(i));
                end
            end
        end

        if isfield(handles, 'headingArrows')
            for i = 1:length(handles.headingArrows)
                if ishandle(handles.headingArrows(i))
                    delete(handles.headingArrows(i));
                end
            end
        end
        if isfield(handles, 'leftWheels')
            for i = 1:length(handles.leftWheels)
                if ishandle(handles.leftWheels(i))
                    delete(handles.leftWheels(i));
                end
            end
        end
        if isfield(handles, 'rightWheels')
            for i = 1:length(handles.rightWheels)
                if ishandle(handles.rightWheels(i))
                    delete(handles.rightWheels(i));
                end
            end
        end

        % Vẽ lại
        hold on;
        for i = 1:length(targetData)
            plot(targetData(i).x, targetData(i).y, 'rx', 'MarkerSize', 1.5, 'LineWidth', 0.7);
        end

        for i = 1:length(robots)
            r = robots(i);
            Rmat = [cos(r.theta), -sin(r.theta); sin(r.theta), cos(r.theta)];

            % Vẽ thân tròn
            theta_vals = linspace(0, 2*pi, 100);
            circ_x = ROBOT_RADIUS * cos(theta_vals);
            circ_y = ROBOT_RADIUS * sin(theta_vals);
            circle = Rmat * [circ_x; circ_y];
            handles.robotsPlot(i) = fill(circle(1,:) + r.x, circle(2,:) + r.y, [0.6 0.8 1], 'EdgeColor', 'k', 'LineWidth', 1.5);

            % Vẽ bánh xe trái
            wheel_shape = [-WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                            WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                            WHEEL_LENGTH/2,  WHEEL_WIDTH/2;
                           -WHEEL_LENGTH/2,  WHEEL_WIDTH/2]';
            wheel_left = Rmat * wheel_shape + [r.x - sin(r.theta)*(AXLE_LENGTH/2); r.y + cos(r.theta)*(AXLE_LENGTH/2)];
            handles.leftWheels(i) = fill(wheel_left(1,:), wheel_left(2,:), 'k');

            % Vẽ bánh xe phải
            wheel_right = Rmat * wheel_shape + [r.x + sin(r.theta)*(AXLE_LENGTH/2); r.y - cos(r.theta)*(AXLE_LENGTH/2)];
            handles.rightWheels(i) = fill(wheel_right(1,:), wheel_right(2,:), 'k');

            % Vẽ mũi tên heading
            handles.headingArrows(i) = quiver(r.x, r.y, cos(r.theta)*0.15, sin(r.theta)*0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
        end
        hold off;

        pause(0.01);
    end
    
    if exist('s', 'var')
        clear s;
    end

    guidata(hObject, handles);
    msgbox('Mô phỏng kết thúc! Tất cả robot đã di chuyển đến target.', 'Thông báo');


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
    % Lấy lựa chọn của người dùng từ popupmenu
    contents = cellstr(get(hObject, 'String'));  % Lấy danh sách các lựa chọn trong menu
    selectedOption = contents{get(hObject, 'Value')};  % Lấy giá trị đã chọn

    % Kiểm tra lựa chọn của người dùng và thực hiện hành động tương ứng
    if strcmp(selectedOption, 'Simulation')
        % Cập nhật GUI cho chế độ Simulation
        disp('Chế độ: Simulation');
        set(handles.modeText, 'String', 'Simulation');
    elseif strcmp(selectedOption, 'Real-world')
        % Cập nhật GUI cho chế độ Real-world
        disp('Chế độ: Real-world');
        set(handles.modeText, 'String', 'Real-world');
    end

    % Cập nhật lại handles
    guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function [pwm_left, pwm_right] = map_velocity(left_value, right_value, min_magnitude, max_magnitude)
    % Mặc định max_magnitude nếu không truyền vào
    if nargin < 4
        max_magnitude = 150;
    end

    % Hằng số giới hạn vận tốc
    MAX_VELOCITY = 17.25;
    MIN_VELOCITY = 6.5;

    % Giá trị tuyệt đối của vận tốc
    abs_left = abs(left_value);
    abs_right = abs(right_value);

    % Tính tỷ lệ vận tốc giữa hai bánh
    wheel_ratio = abs_left / (abs_right + 0.001);  % Tránh chia cho 0

    % Lấy min/max vận tốc
    min_speed = min(abs_left, abs_right);
    max_speed = max(abs_left, abs_right);

    % Trường hợp vận tốc quá thấp
    if min_speed < MIN_VELOCITY
        scale_factor = MIN_VELOCITY / min_speed;
        scaled_left = abs_left * scale_factor;
        scaled_right = abs_right * scale_factor;

    % Trường hợp vận tốc quá cao
    elseif max_speed > MAX_VELOCITY
        scale_factor = MAX_VELOCITY / max_speed;
        scaled_left = abs_left * scale_factor;
        scaled_right = abs_right * scale_factor;

    % Trường hợp trong khoảng tối ưu
    else
        scaled_left = abs_left;
        scaled_right = abs_right;
    end

    % Ràng buộc lại trong khoảng an toàn
    scaled_left = max(MIN_VELOCITY, min(scaled_left, MAX_VELOCITY));
    scaled_right = max(MIN_VELOCITY, min(scaled_right, MAX_VELOCITY));

    % Mapping thực nghiệm từ rad/s → PWM
    pwm_left = 9.2654 * scaled_left - 12.624;
    pwm_right = 9.3381 * scaled_right - 10.068;

    % Khôi phục dấu
    if left_value < 0
        pwm_left = -pwm_left;
    end
    if right_value < 0
        pwm_right = -pwm_right;
    end


% Thêm biến global để đếm gói tin
global packet_counter;


function send_all_pwm_commands(cmd)
    global s;  % Khai báo global để sử dụng biến s
    global packet_counter; % Khai báo global để sử dụng biến đếm
    
    % Khởi tạo biến đếm nếu chưa tồn tại
    if isempty(packet_counter)
        packet_counter = 0;
    end
    
    if isempty(s)
        error('Cổng COM chưa được khởi tạo.');
    end
    
    % Tăng biến đếm
    packet_counter = packet_counter + 1;
    
    % Gửi dữ liệu
    writeline(s, cmd);
    
    % In thông tin debug với số gói tin
    fprintf('Gói tin #%d - Đã gửi chuỗi PWM: %s\n', packet_counter, cmd);


function pushbutton5_Callback(hObject, eventdata, handles)
    global stopFlag;  % Khai báo sử dụng biến toàn cục stopFlag

    % Kiểm tra chế độ hiện tại từ popup menu
    contents = cellstr(get(handles.popupmenu1, 'String'));
    selectedMode = contents{get(handles.popupmenu1, 'Value')};
    
    % Lấy text hiện tại của nút
    current_state = get(hObject, 'String');
    
    if strcmp(current_state, 'Stop')
        % Đang chạy -> dừng lại
        if strcmp(selectedMode, 'Simulation')
            % Lưu trạng thái tạm dừng vào handles
            handles.isPaused = true;
            stopFlag = true;  % Đặt cờ stopFlag thành true để dừng mô phỏng
            set(hObject, 'String', 'Continue');  % Đổi nút thành "Continue"
            set(hObject, 'BackgroundColor', [0.8 1 0.8]); % Màu xanh nhạt
        else
            % Xử lý cho chế độ real-world
            try
                global s;
                if ~isempty(s)
                    % Gửi lệnh dừng đến tất cả robot
                    robotData = jsondecode(fileread('current_position.json'));
                    for i = 1:length(robotData)
                        cmd_str = sprintf('%d,0,0;', robotData(i).id);
                        send_all_pwm_commands(cmd_str);
                    end
                end
                
                stopFlag = true;  % Đặt cờ stopFlag thành true
                guidata(hObject, handles);
                
                set(hObject, 'String', 'Continue');
                set(hObject, 'BackgroundColor', [0.8 1 0.8]);
                disp('Robot thực tế đã dừng');
            catch ME
                errordlg(sprintf('Lỗi khi dừng robot: %s', ME.message), 'Lỗi');
            end
        end
    else % Nút hiển thị "Continue"
        % Đang dừng -> tiếp tục
        if strcmp(selectedMode, 'Simulation')
            stopFlag = false;  % Đặt cờ stopFlag thành false để tiếp tục mô phỏng
            set(hObject, 'String', 'Stop');
            set(hObject, 'BackgroundColor', [1 0.8 0.8]); % Màu đỏ nhạt
            disp('Đã tiếp tục mô phỏng');
        else
            % Xử lý cho chế độ real-world tương tự
            stopFlag = false;  % Đặt cờ stopFlag thành false
            guidata(hObject, handles);
            set(hObject, 'String', 'Stop');
            set(hObject, 'BackgroundColor', [1 0.8 0.8]); % Màu đỏ nhạt
        end
    end
    
    guidata(hObject, handles);  % Cập nhật handles




function robotIdInput_Callback(hObject, eventdata, handles)
% hObject    handle to robotIdInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of robotIdInput as text
%        str2double(get(hObject,'String')) returns contents of robotIdInput as a double


% --- Executes during object creation, after setting all properties.
function robotIdInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robotIdInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function robotXInput_Callback(hObject, eventdata, handles)
% hObject    handle to robotXInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of robotXInput as text
%        str2double(get(hObject,'String')) returns contents of robotXInput as a double


% --- Executes during object creation, after setting all properties.
function robotXInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robotXInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function robotYInput_Callback(hObject, eventdata, handles)
% hObject    handle to robotYInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of robotYInput as text
%        str2double(get(hObject,'String')) returns contents of robotYInput as a double


% --- Executes during object creation, after setting all properties.
function robotYInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robotYInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function robotThetaInput_Callback(hObject, eventdata, handles)
% hObject    handle to robotThetaInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of robotThetaInput as text
%        str2double(get(hObject,'String')) returns contents of robotThetaInput as a double


% --- Executes during object creation, after setting all properties.
function robotThetaInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robotThetaInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in addrobot_button.
function addrobot_button_Callback(hObject, eventdata, handles)
    % Lấy thông tin nhập từ các ô nhập liệu
    robotID = str2double(get(handles.robotIdInput, 'String'));
    robotX = str2double(get(handles.robotXInput, 'String'));
    robotY = str2double(get(handles.robotYInput, 'String'));
    robotTheta = str2double(get(handles.robotThetaInput, 'String'));

    % Kiểm tra xem dữ liệu nhập vào có hợp lệ không
    if isempty(robotID) || isempty(robotX) || isempty(robotY) || isempty(robotTheta)
        msgbox('Vui lòng nhập đầy đủ thông tin robot.', 'Lỗi', 'error');
        return;
    end
    
    % Đọc dữ liệu hiện tại từ file JSON
    if isfile('current_position.json')
        robotData = jsondecode(fileread('current_position.json'));
    else
        robotData = [];  % Nếu file không tồn tại, tạo một mảng rỗng
    end
    
    % Thêm robot mới vào danh sách dữ liệu
    newRobot.id = robotID;
    newRobot.x = robotX;
    newRobot.y = robotY;
    newRobot.theta = robotTheta;
    robotData = [robotData; newRobot];  % Thêm robot mới vào cuối mảng
    
    % Cập nhật lại file JSON
    fid = fopen('current_position.json', 'w');
    if fid == -1
        error('Không thể mở file để ghi dữ liệu.');
    end
    fwrite(fid, jsonencode(robotData), 'char');
    fclose(fid);
    
    % Hiển thị thông báo về robot vừa thêm
    msg = sprintf('Đã thêm robot với ID: %d\nX: %.2f, Y: %.2f, Theta: %.2f', ...
        robotID, robotX, robotY, robotTheta);
    msgbox(msg, 'Thông báo', 'help');
    
    % Cập nhật giao diện, nếu cần
    % Thí dụ: cập nhật lại vùng hiển thị robot
    guidata(hObject, handles);  % Cập nhật trạng thái của giao diện


% --- Executes on button press in Clearrobot.
function Clearrobot_Callback(hObject, eventdata, handles)
    % Đặt lại dữ liệu trong file current_position.json thành mảng rỗng
    fid = fopen('current_position.json', 'w');  % Mở file để ghi
    if fid == -1
        error('Không thể mở file để xóa dữ liệu.');
    end
    fwrite(fid, jsonencode([]), 'char');  % Ghi mảng rỗng vào file
    fclose(fid);  % Đóng file
    
    % Cập nhật giao diện để báo cho người dùng biết dữ liệu đã bị xóa
    msgbox('Dữ liệu robot đã bị xóa thành công!', 'Thông báo');
    
    % Cập nhật lại trạng thái của giao diện
    guidata(hObject, handles);  % Cập nhật trạng thái GUI sau khi xóa dữ liệu


% --- Executes during object creation, after setting all properties.
function Clearrobot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Clearrobot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit32_Callback(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit32 as text
%        str2double(get(hObject,'String')) returns contents of edit32 as a double


% --- Executes during object creation, after setting all properties.
function edit32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit33_Callback(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit33 as text
%        str2double(get(hObject,'String')) returns contents of edit33 as a double


% --- Executes during object creation, after setting all properties.
function edit33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function targetXInput_Callback(hObject, eventdata, handles)
% hObject    handle to targetXInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of targetXInput as text
%        str2double(get(hObject,'String')) returns contents of targetXInput as a double


% --- Executes during object creation, after setting all properties.
function targetXInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to targetXInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function targetYInput_Callback(hObject, eventdata, handles)
% hObject    handle to targetYInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of targetYInput as text
%        str2double(get(hObject,'String')) returns contents of targetYInput as a double


% --- Executes during object creation, after setting all properties.
function targetYInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to targetYInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in addtarget.
function addtarget_Callback(hObject, eventdata, handles)
    % Lấy tọa độ từ các ô nhập liệu
    targetX = str2double(get(handles.targetXInput, 'String'));
    targetY = str2double(get(handles.targetYInput, 'String'));
    
    % Kiểm tra xem người dùng đã nhập dữ liệu hợp lệ không
    if isempty(targetX) || isempty(targetY)
        msgbox('Vui lòng nhập đầy đủ tọa độ X và Y!', 'Lỗi', 'error');
        return;
    end
    
    % Đọc dữ liệu hiện tại từ file target_position.json
    if isfile('target_position.json')
        targetData = jsondecode(fileread('target_position.json'));
    else
        targetData = [];  % Nếu file không tồn tại, tạo mảng rỗng
    end
    
    % Thêm điểm đích mới vào dữ liệu
    newTarget.x = targetX;
    newTarget.y = targetY;
    targetData = [targetData; newTarget];  % Thêm vào cuối mảng
    
    % Cập nhật lại file target_position.json
    fid = fopen('target_position.json', 'w');
    if fid == -1
        error('Không thể mở file để ghi dữ liệu.');
    end
    fwrite(fid, jsonencode(targetData), 'char');  % Ghi lại dữ liệu vào file
    fclose(fid);  % Đóng file
    
    % Hiển thị thông báo về điểm đích đã thêm
    msg = sprintf('Đã thêm điểm đích tại (X: %.2f, Y: %.2f)', targetX, targetY);
    msgbox(msg, 'Thông báo', 'help');
    
    % Cập nhật lại giao diện
    guidata(hObject, handles);  % Cập nhật trạng thái GUI sau khi thêm điểm đích


% --- Executes on button press in cleartarget.
function cleartarget_Callback(hObject, eventdata, handles)
    % Mở file target_position.json để xóa dữ liệu
    fid = fopen('target_position.json', 'w');  % Mở file để ghi
    if fid == -1
        error('Không thể mở file để xóa dữ liệu.');
    end
    fwrite(fid, jsonencode([]), 'char');  % Ghi mảng rỗng vào file
    fclose(fid);  % Đóng file
    
    % Hiển thị thông báo cho người dùng biết dữ liệu đã bị xóa
    msgbox('Dữ liệu điểm đích đã bị xóa thành công!', 'Thông báo');
    
    % Cập nhật lại giao diện để phản ánh sự thay đổi
    guidata(hObject, handles);  % Cập nhật trạng thái GUI sau khi xóa dữ liệu


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function edit25_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function edit26_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in clicktarget_button.
function clicktarget_button_Callback(hObject, eventdata, handles)
    % Get the current toggle state of the button
    if ~isfield(handles, 'targetClickMode') || isempty(handles.targetClickMode)
        handles.targetClickMode = false;
    end
    
    % Toggle the state
    handles.targetClickMode = ~handles.targetClickMode;
    
    % Get axes handle
    axesHandle = findobj(handles.figure1, 'Type', 'axes');
    
    if handles.targetClickMode
        % Mode is active - enable clicking
        set(hObject, 'BackgroundColor', [0.8, 1, 0.8]);  % Light green
        set(axesHandle, 'ButtonDownFcn', @(src, eventdata)axes_click_Callback(src, eventdata, handles));
        set(hObject, 'String', 'Stop Adding Targets');
        msgbox('Click Mode Activated: Click on the simulation area to add targets', 'Target Click Mode');
    else
        % Mode is inactive - disable clicking
        set(hObject, 'BackgroundColor', [0.94, 0.94, 0.94]);  % Default color
        set(axesHandle, 'ButtonDownFcn', '');
        set(hObject, 'String', 'Add Target by Click');
    end
    
    % Update handles structure
    guidata(hObject, handles);

% --- Callback for axes clicks when in target creation mode
function axes_click_Callback(hObject, eventdata, handles)
    % Get the click coordinates
    point = get(hObject, 'CurrentPoint');
    x = point(1, 1);
    y = point(1, 2);
    
    % Check if point is within simulation boundaries
    xLim = get(hObject, 'XLim');
    yLim = get(hObject, 'YLim');
    
    if x < xLim(1) || x > xLim(2) || y < yLim(1) || y > yLim(2)
        % Point is outside simulation area
        return;
    end
    
    % Load existing targets from file
    if isfile('target_position.json')
        targetData = jsondecode(fileread('target_position.json'));
    else
        targetData = [];
    end
    
    % Add new target
    newTarget.x = x;
    newTarget.y = y;
    targetData = [targetData; newTarget];
    
    % Save updated targets to file
    fid = fopen('target_position.json', 'w');
    if fid == -1
        error('Không thể mở file để ghi dữ liệu.');
    end
    fwrite(fid, jsonencode(targetData), 'char');
    fclose(fid);
    
    % Plot the new target on the simulation
    hold on;
    plot(x, y, 'rx', 'MarkerSize', 3, 'LineWidth', 1);
    hold off;
    
    % Update target input fields if they exist
    if isfield(handles, 'targetXInput') && ishandle(handles.targetXInput)
        set(handles.targetXInput, 'String', num2str(x));
    end
    if isfield(handles, 'targetYInput') && ishandle(handles.targetYInput)
        set(handles.targetYInput, 'String', num2str(y));
    end
    
    % Display info about the added target
    targetCount = length(targetData);
    fprintf('Đã thêm điểm đích #%d tại (%.2f, %.2f)\n', targetCount, x, y);
