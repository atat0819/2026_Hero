%暑假舵：
%
%
slope_angle = 17; %坡度（度）
slope_length = 1.271; %坡长（米）
slope_height = slope_length * tand(slope_angle); % 斜坡高度
%上面的数据在下面的计算中并没有用上的

l = 0.54;      % 车长 单位：米
k = 0.54;      % 车宽 单位：米  
h = 0.0975;    % 重心高度 单位：米 重心到底盘+1/2轮子半径
m = 14;        % 质量 单位：kg
g = 9.8;       % 重力加速度

beta_range = 0:1:43; %0到43度，步长为1度
A0_values = zeros(size(beta_range));
A1_values = zeros(size(beta_range));
A2_values = zeros(size(beta_range));
A3_values = zeros(size(beta_range));

for i = 1:length(beta_range)
    beta = deg2rad(beta_range(i));
    
    % 重心投影点坐标
    P_center = [h*sin(beta), 0, -h*cos(beta)];

    % 轮子投影点坐标（假设α=0，车辆正对斜坡）
    % 轮子投影点坐标（北东天坐标系）
    % 前右轮：x=+l/2（前），y=+k/2（右），z=-h（底盘平面）
    wheel0 = [l/2, k/2, -h];    
    % 前左轮：x=+l/2（前），y=-k/2（左），z=-h（底盘平面）
    wheel1 = [l/2, -k/2, -h];   
    % 后右轮：x=-l/2（后），y=+k/2（右），z=-h（底盘平面）
    wheel2 = [-l/2, k/2, -h];   
    % 后左轮：x=-l/2（后），y=-k/2（左），z=-h（底盘平面）
    wheel3 = [-l/2, -k/2, -h];  
    
    
        
    % 计算四个距离
    L0 = sqrt((wheel0(1)-P_center(1))^2 + (wheel0(2)-P_center(2))^2 + (wheel0(3)-P_center(3))^2);
    L1 = sqrt((wheel1(1)-P_center(1))^2 + (wheel1(2)-P_center(2))^2 + (wheel1(3)-P_center(3))^2);
    L2 = sqrt((wheel2(1)-P_center(1))^2 + (wheel2(2)-P_center(2))^2 + (wheel2(3)-P_center(3))^2);
    L3 = sqrt((wheel3(1)-P_center(1))^2 + (wheel3(2)-P_center(2))^2 + (wheel3(3)-P_center(3))^2);
    
    % 求解增益系数
    % N0*L0 = N1*L1 = N2*L2 = N3*L3 = k
    % N0 + N1 + N2 + N3 = mg*cos(beta)
    k_value = (m*g*cos(beta)) / (1/L0 + 1/L1 + 1/L2 + 1/L3);
    
    N0 = k_value / L0;
    N1 = k_value / L1;
    N2 = k_value / L2; 
    N3 = k_value / L3;
    
    total_N = N0 + N1 + N2 + N3;
    A0_values(i) = N0 / total_N;
    A1_values(i) = N1 / total_N;
    A2_values(i) = N2 / total_N;
    A3_values(i) = N3 / total_N;
end

% 对每个增益系数进行多项式拟合
p0 = polyfit(beta_range, A0_values, 3);  % 3次多项式拟合
p1 = polyfit(beta_range, A1_values, 3);
p2 = polyfit(beta_range, A2_values, 3);
p3 = polyfit(beta_range, A3_values, 3);

