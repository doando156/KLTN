function varargout = simu_gui(varargin)
global simulationPaused robotsBackup sendingStopCommands showPathButtonPressed;
showPathButtonPressed = false;
% SIMU_GUI MATLAB code for simu_gui.fig
global simulationPaused robotsBackup sendingStopCommands;
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

% Last Modified by GUIDE v2.5 13-May-2025 18:00:13

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


 handles.simulationRunning = false; 
 % Khởi tạo biến toàn cục cho Stop/Continue
 global simulationPaused robotsBackup sendingStopCommands;
 simulationPaused = false;
 robotsBackup = [];
 sendingStopCommands = false;
 % Choose default command line output for simu_gui
 handles.output = hObject;
    
 % ===== THÊM CODE KHỞI TẠO TRỤC TỌA ĐỘ =====
% Tìm trục tọa độ trong GUI
axes_handle = findobj(handles.figure1, 'Type', 'axes');
if ~isempty(axes_handle)
    % Thiết lập màu nền trắng
    set(axes_handle, 'Color', 'w');
    % Thiết lập giới hạn trục X từ -1 đến 5 (thay vì -6 đến 6)
    xlim(axes_handle, [-1 6]);
    % Thiết lập giới hạn trục Y từ -1 đến 4 (thay vì -4 đến 4)
    ylim(axes_handle, [-2 4]);
    % Đặt tỷ lệ bằng nhau giữa X và Y
    axis(axes_handle, 'equal');
    % Vẽ khung viền - chỉnh kích thước của khung viền cho phù hợp
    rectangle('Position', [-1, -2, 7, 6], 'EdgeColor', 'k', 'LineWidth', 2, 'Parent', axes_handle);
end
 

 % Thiết lập callback theo dõi di chuyển chuột
 set(handles.figure1, 'WindowButtonMotionFcn', {@robotMotionCallback, handles.figure1});

% Khởi tạo biến quỹ đạo - Thêm vào cuối hàm simu_gui_OpeningFcn
handles.showPaths = false;  % Ban đầu không hiển thị quỹ đạo
handles.pathHistory = cell(0);  % Lịch sử vị trí các robot
handles.pathPlots = gobjects(0);  % Handle của các đường quỹ đạo đã vẽ
handles.pathColors = [
    0 0.4470 0.7410;  % Xanh dương
    0.8500 0.3250 0.0980;  % Cam
    0.9290 0.6940 0.1250;  % Vàng
    0.4940 0.1840 0.5560;  % Tím
    0.4660 0.6740 0.1880;  % Xanh lá
    0.3010 0.7450 0.9330;  % Xanh nhạt
    0.6350 0.0780 0.1840;  % Đỏ đậm
    0 0.5 0;  % Xanh lá đậm
    1 0 1;    % Hồng
    0.75 0.75 0;  % Olive
];
set(hObject, 'CloseRequestFcn', @simu_gui_CloseRequestFcn);
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
    xlim([-1 6]);         % Thay đổi chiều ngang
    ylim([-2 4]);         % Thay đổi chiều cao
    axis equal;
    rectangle('Position', [-1, -2, 7, 6], 'EdgeColor', 'k', 'LineWidth', 2);

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
                robots(i).theta = cameraData.(ids{i}).theta;
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
        
         set(handles.robotsPlot(i), 'UserData', struct('id', robotData(i).id, 'x', x, 'y', y, 'theta', robotData(i).theta));
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

    % Reset quỹ đạo và trạng thái nút Show Path
    handles.pathHistory = cell(0);
    handles.pathPlots = gobjects(0);
    handles.showPaths = false;
    if isfield(handles, 'showPathButton')
        set(handles.showPathButton, 'String', 'Show Path');
    end

    hold off;
    % Lưu lại handles
    guidata(hObject, handles);

    msgbox('Khởi tạo không gian mô phỏng thành công!', 'Thông báo');



function pushbutton2_Callback(hObject, eventdata, handles)
    handles.simulationRunning = true;
    % Keep the old parameters, global variables
    global cameraData;
    global ids;
    global s;
    % Keep the old parameters, global variables
    global cameraData ids s simulationPaused robotsBackup;  % Thêm biến toàn cục
    % Add a timer start variable and create a display for it
    timerStart = tic; % Start the timer
    totalPausedTime = 0; % Track total time spent in paused state
    pauseStart = 0; % Track when a pause begins
    
    % Create a text object to display the timer
    axesHandle = findobj(handles.figure1, 'Type', 'axes');
    if ~isempty(axesHandle)
        handles.timerDisplay = text(4.5, 3.5, 'Time: 0.0s', ...
            'Parent', axesHandle, ...
            'HorizontalAlignment', 'right', ...
            'VerticalAlignment', 'top', ...
            'FontSize', 10, ...
            'FontWeight', 'bold', ...
            'BackgroundColor', [1 1 1 0.7], ...
            'Margin', 3);
    end

     % Create a text object to display the timer
    axesHandle = findobj(handles.figure1, 'Type', 'axes');
    if ~isempty(axesHandle)
        handles.timerDisplay = text(5.5, 3.5, 'Time: 0.0s', ...
            'Parent', axesHandle, ...
            'HorizontalAlignment', 'right', ...
            'VerticalAlignment', 'top', ...
            'FontSize', 10, ...
            'FontWeight', 'bold', ...
            'BackgroundColor', [1 1 1 0.7], ...
            'Margin', 3);
            
        % Thêm text hiển thị quãng đường
        handles.distanceDisplay = text(4.5, 3.2, 'Distance: 0.0m', ...
            'Parent', axesHandle, ...
            'HorizontalAlignment', 'right', ...
            'VerticalAlignment', 'top', ...
            'FontSize', 10, ...
            'FontWeight', 'bold', ...
            'BackgroundColor', [1 1 1 0.7], ...
            'Margin', 3);
    end
    
    % Constants and parameters
    DATA_FILE_PATH = 'C:/Users/Do Doan/OneDrive/Documents/HocTapUet/DATN_Final/aruco_python/data.json';
    threshold = 0.9;
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
    
    % Chọn leader (robot gần target nhất)
    leader = robotData(1);

    % --- Fix for leader-follower algorithm ---
    % Khởi tạo chuỗi follower
    remaining_followers = robotData(2:end);  % Danh sách các robot còn lại
    ordered_followers = struct(); % Initialize as empty struct with no predefined fields
    current = leader; % Bắt đầu từ leader

    % Xây dựng chuỗi followers
    while ~isempty(remaining_followers)
        % Tính khoảng cách từ tất cả followers còn lại đến robot hiện tại
        distances = zeros(length(remaining_followers), 1);
        for j = 1:length(remaining_followers)
            dx = remaining_followers(j).x - current.x;
            dy = remaining_followers(j).y - current.y;
            distances(j) = sqrt(dx^2 + dy^2);
        end

        % Chọn follower gần nhất
        [~, min_idx] = min(distances);
        
        % Add the selected follower to ordered_followers array
        if isempty(fieldnames(ordered_followers))
            % First assignment - directly assign
            ordered_followers = remaining_followers(min_idx);
        else
            % Subsequently append using vertical concatenation
            ordered_followers = vertcat(ordered_followers, remaining_followers(min_idx));
        end
        
        current = remaining_followers(min_idx); % Cập nhật robot hiện tại

        % Loại bỏ follower đã chọn
        remaining_followers(min_idx) = [];
    end
    % Cập nhật robotData với leader và followers đã sắp xếp
    if isempty(ordered_followers) || ~isfield(ordered_followers, 'id')
        % If there's only one robot, just use the leader
        robotData = leader;
    else
        % If there are multiple robots, concatenate leader with ordered followers
        robotData = [leader; ordered_followers];
    end
    
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
        robots(i).theta = robotData(i).theta*pi / 180; % Chuyển đổi sang radian
        robots(i).targetIndex = 1;  % Gán chỉ số mục tiêu ban đầu cho mỗi robot
    end
    % Lưu mảng robots vào handles để có thể truy cập từ callback khác
    handles.robots = robots;
    handles.pathHistory = cell(length(robots), 1);
    for i = 1:length(robots)
        handles.pathHistory{i} = [robots(i).x, robots(i).y]; % Điểm bắt đầu
    end
    handles.pathPlots = gobjects(length(robots), 1);
    guidata(hObject, handles);

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
        % THÊM DÒNG NÀY: Cho phép MATLAB xử lý các sự kiện GUI đang chờ
        drawnow;   
        
        % Update timer display if it exists
        if isfield(handles, 'timerDisplay') && ishandle(handles.timerDisplay')
            % Calculate elapsed time considering pauses
            elapsedTime = toc(timerStart) - totalPausedTime;
            set(handles.timerDisplay, 'String', sprintf('Time: %.1fs', elapsedTime));
        end

        % Update timer display if it exists
        if isfield(handles, 'timerDisplay') && ishandle(handles.timerDisplay)
            % Calculate elapsed time considering pauses
            elapsedTime = toc(timerStart) - totalPausedTime;
            set(handles.timerDisplay, 'String', sprintf('Time: %.1fs', elapsedTime));
            
            % Tính toán và hiển thị quãng đường di chuyển
            if isfield(handles, 'pathHistory') && ~isempty(handles.pathHistory)
                totalDistance = 0;
                distances = zeros(length(handles.pathHistory), 1);
                
                % Tính toán quãng đường cho từng robot
                for i = 1:length(handles.pathHistory)
                    if ~isempty(handles.pathHistory{i}) && size(handles.pathHistory{i}, 1) > 1
                        path = handles.pathHistory{i};
                        robotDistance = 0;
                        
                        % Tính tổng khoảng cách giữa các điểm liền kề
                        for j = 2:size(path, 1)
                            dx = path(j,1) - path(j-1,1);
                            dy = path(j,2) - path(j-1,2);
                            segmentDist = sqrt(dx^2 + dy^2);
                            robotDistance = robotDistance + segmentDist;
                        end
                        
                        distances(i) = robotDistance;
                        totalDistance = totalDistance + robotDistance;
                    end
                end
                
                % Định dạng hiển thị khoảng cách
                if length(distances) == 1
                    % Trường hợp chỉ có 1 robot
                    distanceStr = sprintf('Distance: %.2fm', distances(1));
                else
                    % Trường hợp nhiều robot - hiển thị khoảng cách leader và tổng
                    leaderDist = distances(1);
                    distanceStr = sprintf('Dist(L): %.2fm | Total: %.2fm', leaderDist, totalDistance);
                end
                
                % Cập nhật hiển thị khoảng cách
                if isfield(handles, 'distanceDisplay') && ishandle(handles.distanceDisplay)
                    set(handles.distanceDisplay, 'String', distanceStr);
                end
            end
        end
        
        

        % Kiểm tra nếu nút Show Path/Hide Path đã được nhấn
        global showPathButtonPressed;
        if showPathButtonPressed
            % Đặt lại cờ
            showPathButtonPressed = false;
            
            % Đảo trạng thái hiển thị quỹ đạo
            handles.showPaths = ~handles.showPaths;
            
            % Cập nhật nội dung nút
            if handles.showPaths
                set(handles.showPathButton, 'String', 'Hide Path');
            else
                set(handles.showPathButton, 'String', 'Show Path');
                
                % Nếu chuyển sang chế độ ẩn, xóa tất cả quỹ đạo đang hiển thị
                if isfield(handles, 'pathPlots')
                    for i = 1:length(handles.pathPlots)
                        if ishandle(handles.pathPlots(i))
                            delete(handles.pathPlots(i));
                            handles.pathPlots(i) = gobjects(1); % Sửa gobject thành gobjects
                        end
                    end
                end
            end
            
            % KHÔNG cập nhật guidata ở đây, chỉ cập nhật ở cuối vòng lặp
            % Bỏ dòng guidata(hObject, handles);
        end
        % Kiểm tra trạng thái dừng
        global simulationPaused robotsBackup sendingStopCommands;
        if simulationPaused
            % Đợi cho đến khi tiếp tục
            % Record when pause starts if not already paused
            if pauseStart == 0
                pauseStart = toc(timerStart);
            end
            
            while simulationPaused
                pause(0.1);  % Tạm dừng 100ms rồi kiểm tra lại
                drawnow;     % Cho phép GUI cập nhật và xử lý sự kiện
            end

            % When resuming, calculate how long we were paused
            if pauseStart > 0
                totalPausedTime = totalPausedTime + (toc(timerStart) - pauseStart);
                pauseStart = 0; % Reset pause start time
            end
            
            % Khi tiếp tục, khôi phục vị trí nếu cần
            if ~isempty(robotsBackup)
                if strcmp(selectedOption, 'Simulation')
                    % Trong chế độ mô phỏng, khôi phục từ backup
                    robots = robotsBackup;
                else
                    % Trong chế độ real-world, đọc lại vị trí từ camera
                    if isfile(DATA_FILE_PATH)
                        try
                            cameraData = jsondecode(fileread(DATA_FILE_PATH));
                            
                            % Cập nhật vị trí thực tế từ camera
                            for i = 1:length(robots)
                                id_str = ['x', num2str(robots(i).id)];
                                if isfield(cameraData, id_str)
                                    robots(i).x = cameraData.(id_str).x / 100;
                                    robots(i).y = cameraData.(id_str).y / 100;
                                    robots(i).theta = cameraData.(id_str).theta * pi / 180;
                                end
                            end
                            
                            disp('Updated robot positions from camera after continuing');
                        catch ME
                            warning('Error reading camera data: %s', ME.message);
                        end
                    end
                end
                
                robotsBackup = [];  % Xóa backup sau khi đã sử dụng
            end
        end
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
                        robots(i) = robot;
                    else
                        finalTime = toc(timerStart) - totalPausedTime;
                        
                        % Nếu đã đến target cuối và follower đã bám sát: kết thúc mô phỏng
                        msgbox(' Mô phỏng kết thúc: Tất cả robot đã đến đích và bám sát!', 'Hoàn thành');
                        return; 
                    end   
                end
            end

            speed_factor = 0.6;
            % Tính vận tốc và tốc độ góc (omega)
            v_simu = min(0.01, distance) * speed_factor; % Giới hạn vận tốc tối đa
            w_simu = angle_diff * 0.05 * speed_factor; % Tính tốc độ góc


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

            % Sau khi cập nhật vị trí tất cả robot
            handles.robots = robots;
            
           
            % LUÔN CẬP NHẬT lịch sử quỹ đạo, bất kể đang hiển thị hay không
            for i = 1:length(robots)
                handles.pathHistory{i} = [handles.pathHistory{i}; robots(i).x, robots(i).y];
            end

            % CHỈ HIỂN THỊ quỹ đạo khi handles.showPaths = true
            if handles.showPaths
                hold on;
                % Kiểm tra và khởi tạo pathPlots nếu cần
                if ~isfield(handles, 'pathPlots') || isempty(handles.pathPlots) || length(handles.pathPlots) < length(robots)
                    handles.pathPlots = gobjects(length(robots), 1);
                end
                
                for i = 1:length(robots)
                    if size(handles.pathHistory{i}, 1) > 1
                        % Chọn màu dựa trên ID robot
                        colorIdx = mod(robots(i).id-1, size(handles.pathColors, 1)) + 1;
                        robotColor = handles.pathColors(colorIdx, :);
                        
                        % Cập nhật hoặc vẽ mới quỹ đạo
                        if ishandle(handles.pathPlots(i))
                            set(handles.pathPlots(i), 'XData', handles.pathHistory{i}(:,1), ...
                                                    'YData', handles.pathHistory{i}(:,2));
                        else
                            handles.pathPlots(i) = plot(handles.pathHistory{i}(:,1), ...
                                                    handles.pathHistory{i}(:,2), ...
                                                    '-', 'Color', robotColor, ...
                                                    'LineWidth', 1.5);
                        end
                    end
                end
                hold off;
            else
                % Khi ẩn quỹ đạo, xóa các đối tượng đồ họa nhưng VẪN GIỮ dữ liệu lịch sử
                if isfield(handles, 'pathPlots')
                    for i = 1:length(handles.pathPlots)
                        if ishandle(handles.pathPlots(i))
                            delete(handles.pathPlots(i));
                            handles.pathPlots(i) = gobjects(1); % Sửa gobject thành gobjects
                        end
                    end
                end
            end

            % ĐẢM BẢO CẬP NHẬT HANDLES SAU KHI CẬP NHẬT QUỸ ĐẠO
            if ishandle(hObject)
                guidata(hObject, handles);
            end

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
                fprintf('distance: %.2f\n',distance);
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
                            robots(i) = robot; 
                        else
                            % Send stop commands to ALL robots
                            for k = 1:length(robots)
                                cmd_str = sprintf('%d,0,0;', robots(k).id);
                                send_all_pwm_commands(cmd_str);
                                pause(0.1); % Small delay between commands for better reliability
                            end
                            finalTime = toc(timerStart) - totalPausedTime;
                            % If reached the final target and followers are close: end simulation
                            msgbox(' Mô phỏng kết thúc: Tất cả robot đã đến đích và bám sát!', 'Hoàn thành');
                            
                            % Tự động lưu dữ liệu quỹ đạo khi mô phỏng kết thúc
                            if isfield(handles, 'pathHistory') && ~isempty(handles.pathHistory)
                                % Tạo thư mục paths nếu chưa tồn tại
                                if ~exist('paths', 'dir')
                                    mkdir('paths');
                                end
                                
                                % Tạo timestamp cho tên file
                                timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                                
                                % Lấy tên kịch bản hiện tại
                                scenarioName = 'unknown';
                                if isfield(handles, 'currentScenario')
                                    scenarioName = handles.currentScenario;
                                    % Lọc bỏ ký tự không hợp lệ trong tên file
                                    scenarioName = regexprep(scenarioName, '[\\/:*?"<>|]', '_');
                                end
                                
                                % Tính tổng quãng đường của mỗi robot
                                totalDistances = zeros(length(handles.pathHistory), 1);
                                for i = 1:length(handles.pathHistory)
                                    if ~isempty(handles.pathHistory{i}) && size(handles.pathHistory{i}, 1) > 1
                                        path = handles.pathHistory{i};
                                        distance = 0;
                                        for j = 2:size(path, 1)
                                            dx = path(j,1) - path(j-1,1);
                                            dy = path(j,2) - path(j-1,2);
                                            distance = distance + sqrt(dx^2 + dy^2);
                                        end
                                        totalDistances(i) = distance;
                                    end
                                end
                                
                                % Lưu từng quỹ đạo robot thành file CSV riêng
                                validPaths = 0;
                                for i = 1:length(handles.pathHistory)
                                    if ~isempty(handles.pathHistory{i}) && size(handles.pathHistory{i}, 1) > 1
                                        % Lấy ID thực của robot nếu có
                                        robotID = i;
                                        if isfield(handles, 'robots') && length(handles.robots) >= i
                                            robotID = handles.robots(i).id;
                                        end
                                        
                                        % Tạo tên file
                                        filename = fullfile('paths', sprintf('robot_%d_%s_%s.csv', robotID, scenarioName, timestamp));
                                        
                                        % Chuẩn bị dữ liệu với header
                                        path_data = handles.pathHistory{i};
                                        
                                        % Tạo file với header
                                        fid = fopen(filename, 'w');
                                        fprintf(fid, 'X,Y\n'); % Header
                                        fclose(fid);
                                        
                                        % Ghi dữ liệu
                                        dlmwrite(filename, path_data, '-append');
                                        validPaths = validPaths + 1;
                                    end
                                end
                                
                                % Lưu tổng hợp thông tin vào một file riêng
                                summaryFile = fullfile('paths', sprintf('summary_%s_%s.csv', scenarioName, timestamp));
                                fid = fopen(summaryFile, 'w');
                                fprintf(fid, 'RobotID,Points,Distance(m)\n');
                                for i = 1:length(handles.pathHistory)
                                    if ~isempty(handles.pathHistory{i})
                                        robotID = i;
                                        if isfield(handles, 'robots') && length(handles.robots) >= i
                                            robotID = handles.robots(i).id;
                                        end
                                        fprintf(fid, '%d,%d,%.4f\n', robotID, size(handles.pathHistory{i}, 1), totalDistances(i));
                                    end
                                end
                                fclose(fid);
                                
                                % Cập nhật thông báo kết thúc với thông tin về file đã lưu
                                msgStr = sprintf('Mô phỏng kết thúc! Tất cả robot đã di chuyển đến target.\n\nĐã lưu %d quỹ đạo robot vào thư mục "paths"', validPaths);
                                msgbox(msgStr, 'Thông báo');
                                return; % Tránh hiển thị thông báo ở cuối hàm
                            end
                            % Cập nhật guidata
                            guidata(hObject, handles);
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
            % LUÔN CẬP NHẬT lịch sử quỹ đạo, bất kể đang hiển thị hay không
            % Cập nhật sau khi đọc dữ liệu từ camera
            for i = 1:length(robots)
                % Kiểm tra và khởi tạo pathHistory nếu cần
                if ~isfield(handles, 'pathHistory') || length(handles.pathHistory) < length(robots)
                    handles.pathHistory = cell(length(robots), 1);
                end
                
                % Thêm điểm mới vào quỹ đạo
                if isempty(handles.pathHistory{i})
                    handles.pathHistory{i} = [robots(i).x, robots(i).y];
                else
                    handles.pathHistory{i} = [handles.pathHistory{i}; robots(i).x, robots(i).y];
                end
            end

            % CHỈ HIỂN THỊ quỹ đạo khi handles.showPaths = true
            if handles.showPaths
                hold on;
                % Kiểm tra và khởi tạo pathPlots nếu cần
                if ~isfield(handles, 'pathPlots') || isempty(handles.pathPlots) || length(handles.pathPlots) < length(robots)
                    handles.pathPlots = gobjects(length(robots), 1);
                end
                
                for i = 1:length(robots)
                    if size(handles.pathHistory{i}, 1) > 1
                        % Chọn màu dựa trên ID robot
                        colorIdx = mod(robots(i).id-1, size(handles.pathColors, 1)) + 1;
                        robotColor = handles.pathColors(colorIdx, :);
                        
                        % Cập nhật hoặc vẽ mới quỹ đạo
                        if ishandle(handles.pathPlots(i))
                            set(handles.pathPlots(i), 'XData', handles.pathHistory{i}(:,1), ...
                                                    'YData', handles.pathHistory{i}(:,2));
                        else
                            handles.pathPlots(i) = plot(handles.pathHistory{i}(:,1), ...
                                                    handles.pathHistory{i}(:,2), ...
                                                    '-', 'Color', robotColor, ...
                                                    'LineWidth', 1.5);
                        end
                    end
                end
                hold off;
            else
                % Khi ẩn quỹ đạo, xóa các đối tượng đồ họa nhưng VẪN GIỮ dữ liệu lịch sử
                if isfield(handles, 'pathPlots')
                    for i = 1:length(handles.pathPlots)
                        if ishandle(handles.pathPlots(i))
                            delete(handles.pathPlots(i));
                            handles.pathPlots(i) = gobjects(1);
                        end
                    end
                end
            end

            % ĐẢM BẢO CẬP NHẬT HANDLES SAU KHI CẬP NHẬT QUỸ ĐẠO
            if ishandle(hObject)
                guidata(hObject, handles);
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
            thetaForUserData = r.theta * 180/pi;
            Rmat = [cos(r.theta), -sin(r.theta); sin(r.theta), cos(r.theta)];

            % Vẽ thân tròn
            theta_vals = linspace(0, 2*pi, 100);
            circ_x = ROBOT_RADIUS * cos(theta_vals);
            circ_y = ROBOT_RADIUS * sin(theta_vals);
            circle = Rmat * [circ_x; circ_y];
            handles.robotsPlot(i) = fill(circle(1,:) + r.x, circle(2,:) + r.y, [0.6 0.8 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
            set(handles.robotsPlot(i), 'UserData', struct('id', robots(i).id, 'x', r.x, 'y', r.y, 'theta', thetaForUserData));
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
        % CẬP NHẬT GUIDATA VÀ LÀM MỚI CALLBACK
        guidata(hObject, handles);
        % Làm mới callback theo dõi di chuyển chuột với handles mới nhất
        set(handles.figure1, 'WindowButtonMotionFcn', {@robotMotionCallback, handles.figure1});

        pause(0.01);
    end

    % Reset biến toàn cục
    global showPathButtonPressed;
    showPathButtonPressed = false;
    global simulationPaused;
    simulationPaused = false;
    if isfield(handles, 'stopbutton') && ishandle(handles.stopbutton)
        set(handles.stopbutton, 'String', 'Stop');
    end
    % Calculate final elapsed time if not already done
    finalTime = toc(timerStart) - totalPausedTime;
    % Clean up timer display
    if isfield(handles, 'timerDisplay') && ishandle(handles.timerDisplay)
        delete(handles.timerDisplay);
    end
    % Clean up timer display
    if isfield(handles, 'timerDisplay') && ishandle(handles.timerDisplay)
        delete(handles.timerDisplay);
    end
    
    % Clean up distance display
    if isfield(handles, 'distanceDisplay') && ishandle(handles.distanceDisplay)
        delete(handles.distanceDisplay);
    end
    
    if exist('s', 'var')
        clear s;
    end

    if isfield(handles, 'pathHistory') && ~isempty(handles.pathHistory)
        assignin('base', 'pathHistory', handles.pathHistory);
        assignin('base', 'robotDistances', totalDistances); % Nếu có tính toán khoảng cách
    end

    handles.simulationRunning = false;
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


% --- Executes on button press in addrobot_button.
function addrobot_button_Callback(hObject, eventdata, handles)
    % Kiểm tra chế độ hiện tại từ popup menu
    contents = cellstr(get(handles.popupmenu1, 'String'));
    selectedOption = contents{get(handles.popupmenu1, 'Value')};
    
    % Chỉ hoạt động trong chế độ Simulation
    if ~strcmp(selectedOption, 'Simulation')
        msgbox('Chức năng này chỉ khả dụng trong chế độ Simulation', 'Thông báo', 'warn');
        return;
    end
    
    % Đọc dữ liệu robot hiện tại
    if isfile('current_position.json')
        robotData = jsondecode(fileread('current_position.json'));
    else
        robotData = [];
    end
    
    % Thiết lập ID mới là số lượng robot hiện có + 1
    if isempty(robotData)
        newID = 1;
    else
        newID = length(robotData) + 1;
    end
    
    % Hiển thị hộp thoại nhập liệu
    prompt = {'Tọa độ X:', 'Tọa độ Y:', 'Góc Theta (độ):'};
    dlgtitle = sprintf('Thêm Robot (ID: %d)', newID);
    dims = [1 50]; % Kích thước hộp thoại
    defaultans = {'0', '0', '0'}; % Giá trị mặc định
    
    % Hiển thị hộp thoại và đọc dữ liệu nhập vào
    answer = inputdlg(prompt, dlgtitle, dims, defaultans);
    
    % Kiểm tra xem người dùng có nhấn Cancel hay không
    if isempty(answer)
        return;
    end
    
    % Chuyển đổi chuỗi thành số
    try
        robotX = str2double(answer{1});
        robotY = str2double(answer{2});
        robotTheta = str2double(answer{3});
        
        % Kiểm tra xem dữ liệu có hợp lệ không
        if isnan(robotX) || isnan(robotY) || isnan(robotTheta)
            msgbox('Dữ liệu không hợp lệ. Vui lòng nhập số!', 'Lỗi', 'error');
            return;
        end
    catch
        msgbox('Lỗi khi xử lý dữ liệu nhập vào!', 'Lỗi', 'error');
        return;
    end
    
    % Thêm robot mới vào danh sách dữ liệu
    newRobot.id = newID;
    newRobot.x = robotX;
    newRobot.y = robotY;
    newRobot.theta = robotTheta;
    robotData = [robotData; newRobot];
    
    % Cập nhật lại file JSON
    fid = fopen('current_position.json', 'w');
    if fid == -1
        error('Không thể mở file để ghi dữ liệu.');
    end
    fwrite(fid, jsonencode(robotData), 'char');
    fclose(fid);
    
    % --- VẼ NGAY ROBOT MỚI THÊM VÀO ---
    % Thông số robot thực tế
    ROBOT_RADIUS = 0.24 / 2;
    AXLE_LENGTH = 0.175;
    WHEEL_WIDTH = 0.01;
    WHEEL_LENGTH = 0.04;
    
    % Chuyển đổi góc từ độ sang radian
    thetaRad = robotTheta * pi / 180;
    
    % Tạo ma trận xoay
    R = [cos(thetaRad), -sin(thetaRad); sin(thetaRad), cos(thetaRad)];
    
    hold on;
    
    % Vẽ thân tròn
    theta_vals = linspace(0, 2*pi, 100);
    circ_x = ROBOT_RADIUS * cos(theta_vals);
    circ_y = ROBOT_RADIUS * sin(theta_vals);
    circle = R * [circ_x; circ_y];
    
    % Tạo chỗ lưu trữ nếu chưa có
    if ~isfield(handles, 'robotsPlot') || length(handles.robotsPlot) < newID
        if isfield(handles, 'robotsPlot')
            oldLength = length(handles.robotsPlot);
            handles.robotsPlot(oldLength+1:newID) = gobjects(newID-oldLength, 1);
        else
            handles.robotsPlot = gobjects(newID, 1);
        end
    end
    if ~isfield(handles, 'headingArrows') || length(handles.headingArrows) < newID
        if isfield(handles, 'headingArrows')
            oldLength = length(handles.headingArrows);
            handles.headingArrows(oldLength+1:newID) = gobjects(newID-oldLength, 1);
        else
            handles.headingArrows = gobjects(newID, 1);
        end
    end
    if ~isfield(handles, 'leftWheels') || length(handles.leftWheels) < newID
        if isfield(handles, 'leftWheels')
            oldLength = length(handles.leftWheels);
            handles.leftWheels(oldLength+1:newID) = gobjects(newID-oldLength, 1);
        else
            handles.leftWheels = gobjects(newID, 1);
        end
    end
    if ~isfield(handles, 'rightWheels') || length(handles.rightWheels) < newID
        if isfield(handles, 'rightWheels')
            oldLength = length(handles.rightWheels);
            handles.rightWheels(oldLength+1:newID) = gobjects(newID-oldLength, 1);
        else
            handles.rightWheels = gobjects(newID, 1);
        end
    end
    
    % Vẽ thân robot
    handles.robotsPlot(newID) = fill(circle(1,:) + robotX, circle(2,:) + robotY, [0.6 0.8 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
    set(handles.robotsPlot(newID), 'UserData', struct('id', newID, 'x', robotX, 'y', robotY, 'theta', robotTheta));
    % Vẽ bánh trái
    wheel_shape = [-WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                    WHEEL_LENGTH/2, -WHEEL_WIDTH/2;
                    WHEEL_LENGTH/2,  WHEEL_WIDTH/2;
                   -WHEEL_LENGTH/2,  WHEEL_WIDTH/2]';
    wheel_left = R * wheel_shape + [robotX - sin(thetaRad)*(AXLE_LENGTH/2); robotY + cos(thetaRad)*(AXLE_LENGTH/2)];
    handles.leftWheels(newID) = fill(wheel_left(1,:), wheel_left(2,:), 'k');
    
    % Vẽ bánh phải
    wheel_right = R * wheel_shape + [robotX + sin(thetaRad)*(AXLE_LENGTH/2); robotY - cos(thetaRad)*(AXLE_LENGTH/2)];
    handles.rightWheels(newID) = fill(wheel_right(1,:), wheel_right(2,:), 'k');
    
    % Vẽ mũi tên heading
    handles.headingArrows(newID) = quiver(robotX, robotY, cos(thetaRad)*0.15, sin(thetaRad)*0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    
    hold off;
    
    % Hiển thị thông báo về robot vừa thêm
    msg = sprintf('Đã thêm robot với ID: %d\nX: %.2f, Y: %.2f, Theta: %.2f', ...
        newID, robotX, robotY, robotTheta);
    msgbox(msg, 'Thông báo', 'help');
    
    % Cập nhật lại handles
    guidata(hObject, handles);



% --- Executes on button press in addtarget.
function addtarget_Callback(hObject, eventdata, handles)
    % Hiển thị hộp thoại nhập liệu tọa độ X và Y
    prompt = {'Tọa độ X:', 'Tọa độ Y:'};
    dlgtitle = 'Nhập tọa độ điểm đích';
    dims = [1 50]; % Kích thước hộp thoại
    defaultans = {'0', '0'}; % Giá trị mặc định
    
    % Hiển thị hộp thoại và đọc dữ liệu nhập vào
    answer = inputdlg(prompt, dlgtitle, dims, defaultans);
    
    % Kiểm tra xem người dùng có nhấn Cancel hay không
    if isempty(answer)
        return;
    end
    
    % Chuyển đổi chuỗi thành số
    try
        targetX = str2double(answer{1});
        targetY = str2double(answer{2});
        
        % Kiểm tra xem dữ liệu có hợp lệ không
        if isnan(targetX) || isnan(targetY)
            msgbox('Dữ liệu không hợp lệ. Vui lòng nhập số!', 'Lỗi', 'error');
            return;
        end
    catch
        msgbox('Lỗi khi xử lý dữ liệu nhập vào!', 'Lỗi', 'error');
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
    fwrite(fid, jsonencode(targetData), 'char');
    fclose(fid);
    
    % VẼ NGAY LẬP TỨC TARGET MỚI VỪA THÊM VÀO
    % Tìm đối tượng axes hiện tại
    axesHandle = findobj(handles.figure1, 'Type', 'axes');
    if ~isempty(axesHandle)
        % Giữ nguyên nội dung hiện tại của axes
        axes(axesHandle);
        hold on;
        % Vẽ mục tiêu mới với marker 'x' màu đỏ
        plot(targetX, targetY, 'rx', 'MarkerSize', 3, 'LineWidth', 1);
        hold off;
    end
    
    % Hiển thị thông báo về điểm đích vừa thêm
    targetCount = length(targetData);
    msg = sprintf('Đã thêm điểm đích #%d tại (X: %.2f, Y: %.2f)', targetCount, targetX, targetY);
    msgbox(msg, 'Thông báo', 'help');
    
    % Cập nhật lại giao diện
    guidata(hObject, handles);



% --- Executes on button press in cleartarget.
function cleartarget_Callback(hObject, eventdata, handles)
    % Mở file target_position.json để xóa dữ liệu
    fid = fopen('target_position.json', 'w');  % Mở file để ghi
    if fid == -1
        error('Không thể mở file để xóa dữ liệu.');
    end
    fwrite(fid, jsonencode([]), 'char');  % Ghi mảng rỗng vào file
    fclose(fid);  % Đóng file
    
    % XÓA CÁC TARGET TRÊN MÀN HÌNH
    % Tìm trục đồ thị hiện tại
    ax = findobj(handles.figure1, 'Type', 'axes');
    
    if ~isempty(ax)
        % Tìm tất cả các đối tượng Line trên đồ thị
        allLines = findobj(ax, 'Type', 'line');
        
        % Lọc ra các đối tượng là target (đánh dấu 'rx')
        for i = 1:length(allLines)
            % Kiểm tra marker style và marker color
            if strcmp(get(allLines(i), 'Marker'), 'x') && ...
               isequal(get(allLines(i), 'Color'), [1 0 0])  % Màu đỏ = [1 0 0]
                delete(allLines(i));  % Xóa target marker
            end
        end
    end
    
    % Hiển thị thông báo cho người dùng biết dữ liệu đã bị xóa
    msgbox('Dữ liệu điểm đích đã bị xóa thành công!', 'Thông báo');
    
    % Cập nhật lại giao diện để phản ánh sự thay đổi
    guidata(hObject, handles);  % Cập nhật trạng thái GUI sau khi xóa dữ liệu



% --- Executes on button press in Clearrobot.
function Clearrobot_Callback(hObject, eventdata, handles)
    % Đặt lại dữ liệu trong file current_position.json thành mảng rỗng
    fid = fopen('current_position.json', 'w');  % Mở file để ghi
    if fid == -1
        error('Không thể mở file để xóa dữ liệu.');
    end
    fwrite(fid, jsonencode([]), 'char');  % Ghi mảng rỗng vào file
    fclose(fid);  % Đóng file
    
    % XÓA CÁC ROBOT TRÊN MÀN HÌNH
    ax = findobj(handles.figure1, 'Type', 'axes');
    
    if ~isempty(ax)
        % Xóa các thành phần robot: thân, mũi tên, và bánh xe
        if isfield(handles, 'robotsPlot')
            for i = 1:length(handles.robotsPlot)
                if ishandle(handles.robotsPlot(i))
                    delete(handles.robotsPlot(i));  % Xóa thân robot
                end
            end
        end

        if isfield(handles, 'headingArrows')
            for i = 1:length(handles.headingArrows)
                if ishandle(handles.headingArrows(i))
                    delete(handles.headingArrows(i));  % Xóa mũi tên
                end
            end
        end

        if isfield(handles, 'leftWheels')
            for i = 1:length(handles.leftWheels)
                if ishandle(handles.leftWheels(i))
                    delete(handles.leftWheels(i));  % Xóa bánh trái
                end
            end
        end

        if isfield(handles, 'rightWheels')
            for i = 1:length(handles.rightWheels)
                if ishandle(handles.rightWheels(i))
                    delete(handles.rightWheels(i));  % Xóa bánh phải
                end
            end
        end
        
        % Cập nhật lại handles để phản ánh rằng không còn robot nào
        handles.robotsPlot = [];
        handles.headingArrows = [];
        handles.leftWheels = [];
        handles.rightWheels = [];
    end

    % Reset quỹ đạo và trạng thái nút Show Path
    handles.pathHistory = cell(0);
    handles.pathPlots = gobjects(0);
    handles.showPaths = false;
    if isfield(handles, 'showPathButton')
        set(handles.showPathButton, 'String', 'Show Path');
    end
    
    % Cập nhật giao diện để báo cho người dùng biết dữ liệu đã bị xóa
    msgbox('Dữ liệu robot đã bị xóa thành công!', 'Thông báo');
    
    % Cập nhật lại trạng thái của giao diện
    guidata(hObject, handles);  % Cập nhật trạng thái GUI sau khi xóa dữ liệu

% --- Executes during object creation, after setting all properties.
function Clearrobot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Clearrobot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: pushbutton controls usually have a white background on Windows.
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


% --- Executes on button press in stopbutton.
function stopbutton_Callback(hObject, eventdata, handles)
    global simulationPaused robotsBackup sendingStopCommands;
    % Xác định chế độ hiện tại từ popupmenu
    contents = cellstr(get(handles.popupmenu1, 'String'));
    selectedOption = contents{get(handles.popupmenu1, 'Value')};
    isRealWorld = strcmp(selectedOption, 'Real-world');
    % Nếu nút đang hiển thị "Stop"
    if strcmp(get(hObject, 'String'), 'Stop')
        % Đổi sang trạng thái dừng
        simulationPaused = true;
        set(hObject, 'String', 'Continue');
        
        % Sao lưu vị trí robot hiện tại
        robotsBackup = handles.robots;
         % Nếu đang ở chế độ real-world, gửi lệnh dừng cho tất cả robot
         if isRealWorld
            % Tạo một biến global để đánh dấu đang gửi lệnh dừng
            global sendingStopCommands;
            sendingStopCommands = true;
            
            % Bắt đầu một timer để liên tục gửi lệnh dừng
            global stopTimer;
            if ~isempty(timerfind('Name', 'StopCommandTimer'))
                stop(stopTimer);
                delete(stopTimer);
            end
            
            stopTimer = timer('Name', 'StopCommandTimer', ...
                             'Period', 0.5, ... % Gửi lệnh mỗi 0.5 giây
                             'ExecutionMode', 'fixedRate', ...
                             'TimerFcn', {@sendStopCommands, handles});
            start(stopTimer);
        end    
    else
        % Đổi sang trạng thái tiếp tục
        simulationPaused = false;
        set(hObject, 'String', 'Stop');
        
         % Nếu đang ở chế độ real-world, dừng việc gửi lệnh dừng
         if isRealWorld
            global sendingStopCommands stopTimer;
            sendingStopCommands = false;
            
            % Dừng và xóa timer nếu đang chạy
            if ~isempty(timerfind('Name', 'StopCommandTimer'))
                stop(stopTimer);
                delete(stopTimer);
            end
        end
        
        % THÊM: Đảm bảo focus trở lại axes chính
        axesHandle = findobj(handles.figure1, 'Type', 'axes');
        if ~isempty(axesHandle)
            axes(axesHandle); % Đặt axes hiện hành là axes chính
        end
    end
    
    guidata(hObject, handles);

function robotMotionCallback(src, ~, fig)
    % Lấy handles hiện tại 
    handles = guidata(fig);
    
    % Lấy vị trí hiện tại của con trỏ chuột
    axesHandle = findobj(fig, 'Type', 'axes');
    if isempty(axesHandle) || ~ishandle(axesHandle)
        return;
    end
    
    % Chuyển đổi tọa độ chuột sang tọa độ axes
    cp = get(axesHandle, 'CurrentPoint');
    x = cp(1, 1);
    y = cp(1, 2);
    
    % Lấy kích thước của axes
    xLim = get(axesHandle, 'XLim');
    yLim = get(axesHandle, 'YLim');
    
    % Kiểm tra xem chuột có nằm trong axes không
    if x < xLim(1) || x > xLim(2) || y < yLim(1) || y > yLim(2)
        % Xóa tooltip nếu có
        hTip = findobj(fig, 'Tag', 'RobotInfoTip');
        if ~isempty(hTip)
            delete(hTip);
        end
        return;
    end
    
    % Kiểm tra từng robot
    if isfield(handles, 'robotsPlot')
        for i = 1:length(handles.robotsPlot)
            if ishandle(handles.robotsPlot(i))
                % Lấy dữ liệu robot từ UserData
                robotData = get(handles.robotsPlot(i), 'UserData');
                if ~isstruct(robotData)
                    continue;
                end
                
                % Tính khoảng cách từ con trỏ đến tâm robot
                ROBOT_RADIUS = 0.24 / 2;  % Bán kính robot
                dist = sqrt((x - robotData.x)^2 + (y - robotData.y)^2);
                
                % Nếu con trỏ nằm trong robot
                if dist <= ROBOT_RADIUS
                    % Sử dụng phương pháp text trong axes thay vì annotation
                    hTip = findobj(axesHandle, 'Tag', 'RobotInfoTip');
                    if isempty(hTip)
                        % Tạo text mới
                        hTip = text(0, 0, '', 'Parent', axesHandle, ...
                                    'BackgroundColor', [1 1 0.8], ...
                                    'EdgeColor', 'k', ...
                                    'Tag', 'RobotInfoTip', ...
                                    'FontSize', 8, ...
                                    'HorizontalAlignment', 'center', ...
                                    'VerticalAlignment', 'bottom', ...
                                    'Margin', 3);
                    end
                    
                    % Chuyển đổi góc để hiển thị đúng trong tooltip
                    % *** FIX: Xử lý góc theta đúng dựa vào cách lưu trữ ***
                    if isfield(robotData, 'theta')
                        % Không cần chuyển đổi vì lưu trữ đã đúng định dạng độ
                        thetaDegrees = mod(robotData.theta, 360);
                    else
                        thetaDegrees = 0;
                    end
                    
                    % Định vị text ngay phía trên robot
                    set(hTip, 'Position', [robotData.x, robotData.y + ROBOT_RADIUS + 0.05, 0]);
                    
                    % Cập nhật nội dung text
                    set(hTip, 'String', {
                        ['ID: ', num2str(robotData.id)],
                        ['X: ', num2str(robotData.x, '%.2f')],
                        ['Y: ', num2str(robotData.y, '%.2f')],
                        ['Theta: ', num2str(thetaDegrees, '%.1f'), '°']
                    });
                    return;
                end
            end
        end
    end
    
    % Nếu không có robot nào được trỏ đến, xóa tooltip
    hTip = findobj(axesHandle, 'Tag', 'RobotInfoTip');
    if ~isempty(hTip)
        delete(hTip);
    end


% --- Executes on button press in showPathButton.
function showPathButton_Callback(hObject, eventdata, handles)
    global showPathButtonPressed;
    
    % Kiểm tra xem mô phỏng có đang chạy không
    if ~isfield(handles, 'simulationRunning') || ~handles.simulationRunning
        % Mô phỏng KHÔNG chạy - xử lý trực tiếp tại đây
        
        % Đảo trạng thái hiển thị quỹ đạo
        handles.showPaths = ~handles.showPaths;
        
        % Tìm axes chính
        axesHandle = findobj(handles.figure1, 'Type', 'axes');
        if ~isempty(axesHandle)
            axes(axesHandle);
        end
        
        if handles.showPaths
            % Chuyển sang chế độ HIỆN quỹ đạo
            set(hObject, 'String', 'Hide Path');
            
            % Vẽ quỹ đạo nếu có
            if isfield(handles, 'pathHistory') && ~isempty(handles.pathHistory)
                hold on;
                % Khởi tạo pathPlots nếu cần
                if ~isfield(handles, 'pathPlots') || length(handles.pathPlots) < length(handles.pathHistory)
                    handles.pathPlots = gobjects(length(handles.pathHistory), 1);
                end
                
                for i = 1:length(handles.pathHistory)
                    if ~isempty(handles.pathHistory{i}) && size(handles.pathHistory{i}, 1) > 1
                        % Chọn màu dựa trên vị trí trong mảng
                        colorIdx = mod(i-1, size(handles.pathColors, 1)) + 1;
                        robotColor = handles.pathColors(colorIdx, :);
                        
                        % Vẽ đường quỹ đạo
                        handles.pathPlots(i) = plot(handles.pathHistory{i}(:,1), ...
                                               handles.pathHistory{i}(:,2), ...
                                               '-', 'Color', robotColor, ...
                                               'LineWidth', 1.5);
                    end
                end
                hold off;
            end
        else
            % Chuyển sang chế độ ẨN quỹ đạo
            set(hObject, 'String', 'Show Path');
            
            % Xóa các đường quỹ đạo đã vẽ
            if isfield(handles, 'pathPlots')
                for i = 1:length(handles.pathPlots)
                    if ishandle(handles.pathPlots(i))
                        delete(handles.pathPlots(i));
                        handles.pathPlots(i) = gobjects(1);
                    end
                end
            end
        end
        
        % Cập nhật handles
        guidata(hObject, handles);
        
        % Cập nhật ngay lập tức
        drawnow;
    else
        % Mô phỏng đang chạy - sử dụng cơ chế flag như trước
        showPathButtonPressed = true;
        drawnow;
    end


function sendStopCommands(obj, event, handles)
    global sendingStopCommands;
    
    % Nếu không còn ở trạng thái dừng, dừng timer
    if ~sendingStopCommands
        stop(obj);
        return;
    end

    % Lấy handles mới nhất
    handles = guidata(fig);
    
    % Lấy danh sách robot từ handles
    if isfield(handles, 'robots') && ~isempty(handles.robots)
        robots = handles.robots;
        
        % Gửi lệnh dừng cho tất cả robot
        for i = 1:length(robots)
            cmd_str = sprintf('%d,0,0;', robots(i).id);
            try
                send_all_pwm_commands(cmd_str);
                fprintf('Sent stop command to robot %d\n', robots(i).id);
                pause(0.05); % Thêm giãn cách nhỏ giữa các lệnh
            catch ME
                warning('Failed to send stop command: %s', ME.message);
            end
        end
    end

% --- Executes when the figure is closed.
function simu_gui_CloseRequestFcn(hObject, eventdata, handles)
    % Dừng và xóa tất cả timer
    if ~isempty(timerfind('Name', 'StopCommandTimer'))
        global stopTimer;
        stop(stopTimer);
        delete(stopTimer);
    end
    
    % Xóa tài nguyên serialport nếu đang mở
    global s;
    if ~isempty(s) && isvalid(s)
        clear s;
    end
    
    % Đóng figure
    delete(hObject);


% --- Executes on button press in Select.
function Select_Callback(hObject, eventdata, handles)
   % Let the user select a scenario file (JSON format)
    [filename, pathname] = uigetfile('*.json', 'Select a scenario file');
    
    % If user canceled the dialog
    if isequal(filename, 0) || isequal(pathname, 0)
        return;
    end
    
    % Construct full file path
    fullFilePath = fullfile(pathname, filename);
    
    
    % Read the scenario file
    try
        scenarioData = jsondecode(fileread(fullFilePath));
    catch ME
        errordlg(['Error reading scenario file: ' ME.message], 'File Read Error');
        return;
    end
    
    % Make sure file contains required data
    if ~isfield(scenarioData, 'robots') || ~isfield(scenarioData, 'targets')
        errordlg('Invalid scenario file format. File must contain "robots" and "targets" fields.', 'Format Error');
        return;
    end

    % Lưu tên kịch bản vào handles (thêm dòng này sau khi đọc file)
    [~, scenarioName, ~] = fileparts(filename); % Lấy tên file không có phần mở rộng
    handles.currentScenario = scenarioName;
    guidata(hObject, handles); % Cập nhật handles
    
    % Extract robot data
    robotData = scenarioData.robots;
    
    % Extract target data
    targetData = scenarioData.targets;
    
    % Save robot data to current_position.json
    try
        fid = fopen('current_position.json', 'w');
        if fid == -1
            error('Cannot open current_position.json for writing');
        end
        fprintf(fid, '%s', jsonencode(robotData));
        fclose(fid);
    catch ME
        errordlg(['Error saving robot data: ' ME.message], 'Save Error');
        return;
    end
    
    % Save target data to target_position.json
    try
        fid = fopen('target_position.json', 'w');
        if fid == -1
            error('Cannot open target_position.json for writing');
        end
        fprintf(fid, '%s', jsonencode(targetData));
        fclose(fid);
    catch ME
        errordlg(['Error saving target data: ' ME.message], 'Save Error');
        return;
    end
    
    % Show success message with scenario information
    robotCount = length(robotData);
    targetCount = length(targetData);
    msgbox(sprintf('Scenario "%s" loaded successfully!\n- %d robots\n- %d targets\n\nPress "Init" to visualize.', ...
           filename, robotCount, targetCount), 'Scenario Loaded');


% --- Executes on button press in save_scenario.
function save_scenario_Callback(hObject, eventdata, handles)
    % Determine the current mode (Simulation or Real-world)
    contents = cellstr(get(handles.popupmenu1, 'String'));
    selectedOption = contents{get(handles.popupmenu1, 'Value')};
    isSimulation = strcmp(selectedOption, 'Simulation');
    
    % Initialize a structure to hold all scenario data
    scenarioData = struct('robots', [], 'targets', []);
    
    % Get target data - this is the same for both modes
    try
        if isfile('target_position.json')
            targetData = jsondecode(fileread('target_position.json'));
            scenarioData.targets = targetData;
        else
            warndlg('Target position file not found. No targets will be saved.', 'Warning');
            scenarioData.targets = [];
        end
    catch ME
        errordlg(['Error reading target data: ' ME.message], 'Read Error');
        return;
    end
    
    % Get robot data based on the current mode
    if isSimulation
        % Simulation mode - read from current_position.json
        try
            if isfile('current_position.json')
                robotData = jsondecode(fileread('current_position.json'));
                scenarioData.robots = robotData;
            else
                warndlg('Current position file not found. No robot data will be saved.', 'Warning');
                scenarioData.robots = [];
            end
        catch ME
            errordlg(['Error reading robot data: ' ME.message], 'Read Error');
            return;
        end
    else
        % Real-world mode - read from data.json (camera data)
        DATA_FILE_PATH = 'C:/Users/Do Doan/OneDrive/Documents/HocTapUet/DATN_Final/aruco_python/data.json';
        try
            if isfile(DATA_FILE_PATH)
                % Read camera data
                cameraData = jsondecode(fileread(DATA_FILE_PATH));
                
                % Convert camera data format to match scenario format
                robotData = [];
                ids = fieldnames(cameraData);
                
                for i = 1:length(ids)
                    % Extract numeric ID from field name
                    clean_id = regexp(ids{i}, '\d+', 'match');
                    if isempty(clean_id)
                        continue;
                    end
                    robot_id = str2double(clean_id{1});
                    
                    % Create robot entry
                    robot = struct();
                    robot.id = robot_id;
                    robot.x = cameraData.(ids{i}).x / 100;  % Convert from cm to m
                    robot.y = cameraData.(ids{i}).y / 100;  % Convert from cm to m
                    robot.theta = cameraData.(ids{i}).theta;  % Keep angle in degrees
                    
                    % Add to robot array
                    if isempty(robotData)
                        robotData = robot;
                    else
                        robotData(end+1) = robot;
                    end
                end
                
                scenarioData.robots = robotData;
            else
                errordlg(['Camera data file not found at: ' DATA_FILE_PATH], 'File Not Found');
                return;
            end
        catch ME
            errordlg(['Error reading camera data: ' ME.message], 'Read Error');
            return;
        end
    end
    
    % Show save dialog
    [filename, pathname] = uiputfile('*.json', 'Save Scenario As');
    
    % If user canceled the dialog
    if isequal(filename, 0) || isequal(pathname, 0)
        return;
    end
    
    % Construct full file path
    fullFilePath = fullfile(pathname, filename);
    
    % Save scenario data to file
    try
        % Convert to JSON with pretty formatting (indentation)
        jsonStr = jsonencode(scenarioData);
        % Make it more readable with proper line breaks and indentation
        jsonStr = strrep(jsonStr, ',', sprintf(',\n'));
        jsonStr = strrep(jsonStr, '{', sprintf('{\n'));
        jsonStr = strrep(jsonStr, '}', sprintf('\n}'));
        jsonStr = strrep(jsonStr, ':[', sprintf(':[\n'));
        jsonStr = strrep(jsonStr, ']', sprintf('\n]'));
        
        % Write to file
        fid = fopen(fullFilePath, 'w');
        if fid == -1
            error('Cannot open file for writing');
        end
        fprintf(fid, '%s', jsonStr);
        fclose(fid);
        
        % Show success message
        robotCount = length(scenarioData.robots);
        targetCount = length(scenarioData.targets);
        msgbox(sprintf('Scenario saved successfully!\n- %d robots\n- %d targets\n\nSaved to: %s', ...
               robotCount, targetCount, fullFilePath), 'Save Successful');
    catch ME
        errordlg(['Error saving scenario: ' ME.message], 'Save Error');
    end


% --- Executes on button press in export_path_button.
function export_path_button_Callback(hObject, eventdata, handles)
    try
        % Kiểm tra xem có dữ liệu quỹ đạo không
        if ~isfield(handles, 'pathHistory') || isempty(handles.pathHistory)
            % Thử lấy từ workspace nếu có
            workspace_vars = evalin('base', 'who');
            if ismember('pathHistory', workspace_vars)
                pathHistory = evalin('base', 'pathHistory');
                if isempty(pathHistory)
                    errordlg('Không có dữ liệu quỹ đạo để xuất!', 'Lỗi');
                    return;
                end
            else
                errordlg('Không có dữ liệu quỹ đạo để xuất. Hãy chạy mô phỏng trước!', 'Lỗi');
                return;
            end
        else
            pathHistory = handles.pathHistory;
        end
        
        % Tạo thư mục paths nếu chưa tồn tại
        if ~exist('paths', 'dir')
            mkdir('paths');
        end
        
        % Tạo timestamp cho tên file
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        
        % Lấy tên kịch bản hiện tại
        scenarioName = 'unknown';
        if isfield(handles, 'currentScenario') && ~isempty(handles.currentScenario)
            scenarioName = handles.currentScenario;
            % Lọc bỏ ký tự không hợp lệ trong tên file
            scenarioName = regexprep(scenarioName, '[\\/:*?"<>|]', '_');
        end
        
        % Tính tổng quãng đường của mỗi robot
        totalDistances = zeros(length(pathHistory), 1);
        for i = 1:length(pathHistory)
            if ~isempty(pathHistory{i}) && size(pathHistory{i}, 1) > 1
                path = pathHistory{i};
                distance = 0;
                for j = 2:size(path, 1)
                    dx = path(j,1) - path(j-1,1);
                    dy = path(j,2) - path(j-1,2);
                    distance = distance + sqrt(dx^2 + dy^2);
                end
                totalDistances(i) = distance;
            end
        end
        
        % Lưu từng quỹ đạo robot thành file CSV riêng
        validPaths = 0;
        for i = 1:length(pathHistory)
            if ~isempty(pathHistory{i}) && size(pathHistory{i}, 1) > 1
                % Lấy ID thực của robot nếu có
                robotID = i;
                if isfield(handles, 'robots') && length(handles.robots) >= i
                    robotID = handles.robots(i).id;
                end
                
                % Tạo tên file
                filename = fullfile('paths', sprintf('robot_%d_%s_%s.csv', robotID, scenarioName, timestamp));
                
                % Ghi dữ liệu với header
                fid = fopen(filename, 'w');
                fprintf(fid, 'X,Y\n'); % Header
                fclose(fid);
                
                % Ghi dữ liệu
                dlmwrite(filename, pathHistory{i}, '-append');
                validPaths = validPaths + 1;
            end
        end
        
        % Lưu tổng hợp thông tin vào một file riêng
        if validPaths > 0
            summaryFile = fullfile('paths', sprintf('summary_%s_%s.csv', scenarioName, timestamp));
            fid = fopen(summaryFile, 'w');
            fprintf(fid, 'RobotID,Points,Distance(m)\n');
            for i = 1:length(pathHistory)
                if ~isempty(pathHistory{i})
                    robotID = i;
                    if isfield(handles, 'robots') && length(handles.robots) >= i
                        robotID = handles.robots(i).id;
                    end
                    fprintf(fid, '%d,%d,%.4f\n', robotID, size(pathHistory{i}, 1), totalDistances(i));
                end
            end
            fclose(fid);
            
            % Hiển thị thông báo thành công với chi tiết
            msgStr = sprintf('Đã xuất thành công %d quỹ đạo robot!\n\nThư mục: paths\nTổng quãng đường: %.2f m', validPaths, sum(totalDistances));
            msgbox(msgStr, 'Xuất quỹ đạo thành công');
        else
            warndlg('Không có quỹ đạo hợp lệ để xuất!', 'Cảnh báo');
        end
    catch ME
        errordlg(['Lỗi khi xuất quỹ đạo: ' ME.message], 'Lỗi');
    end