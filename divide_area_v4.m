function [ Area_rst ] = divide_area_v4( Core_flag, init_grid, robot_num )
% Area_flag:L*W (L:length of SOI, W:width of SOI, Area_flag(i,j)=1 means free space/0 means obstacles)
% init_pos:robot_num*2
% Area_rst:L*W*robot_num (Area_rst(:,:,i) means the area for robot i)
%% idea
% 不用欧拉距离作为评判的标准，改用向量距离。采用区域向量避开已有无人机的位置？区域向量保证连通性？若不连通应当怎样收敛回来？
% original method: DARP-103行
%% prepare control points
Core_flag = Core_flag-1; %Area_flag(i,j)=0 means free space/-1 means obstacles
L = size(Core_flag,2); %length = cols = x
W = size(Core_flag,1); %width = rols = y
%if mod(L,2)~=0 || mod(W,2)~=0
%    Area_rst = -1;
%    fprintf('area error\n')
%    return
%end
F = L*W;
m = ones(1,robot_num);
E = zeros(W,L,robot_num);
K = zeros(W,L);

%S = zeros(1,robot_num);

%display_area(Area_flag,1);

%% Generate the first matrix
S = zeros(1,robot_num);
for i=1:L
    for j=1:W
        if Core_flag(j,i)==-1
            ob = ob + 1;
            continue;
        end

        for k=1:robot_num
            E(j,i,k) = m(k) * sqrt((i-init_grid(k,1))^2+(j-init_grid(k,2))^2);
        end
        [~,index] = min(E(j,i,:));
        K(j,i) = index;
        S(index) = S(index)+1;
    end
end
figure(2);
handle2 = display_area(K,robot_num,2);
K;
S;
pause();

%% main loop
stop = 0;
while stop==0
    S = zeros(1,robot_num); %total grid number for each robot
    ob = 0;
    count = 0;
        
    %% contour detection for each robot
    Kk = zeros(W,L,robot_num);
    for i=1:L
        for j=1:W
            Kk(j,i,K(j,i)) = 1;
        end
    end
    B = zeros(W,L,robot_num);
    core = [0 1 0; 1 1 1; 0 1 0]*(1/5);
    Ba = zeros(W,L); % contour set; need to be higher variant
    Bi = zeros(W,L,robot_num); % contour note; need to be higher variant
    for k=1:robot_num
       B_t = conv2(Kk(:,:,k),core);
       B_t(W+2,:)=[];B_t(1,:)=[];
       B_t(:,L+2)=[];B_t(:,1)=[];
       %B_t
       B(:,:,k) = B_t;
       
       for i=1:L
           for j=1:W
               if B(j,i,k)>=0.2 && B(j,i,k)<0.8
                  Ba(j,i) = 1;
                  Bi(j,i,k) = 1;
               end
           end
       end
       %Bi(:,:,k)
    end
    %Ba
    
    %% check every grid
    for i=1:L
        for j=1:W
            if Core_flag(j,i)==-1
                ob = ob + 1;
                continue;
            end
            
            if Ba(j,i) ~= 1
                S(K(j,i)) = S(K(j,i)) + 1;
                continue;
            end

            for k=1:robot_num
                %if m(k)<0
                %    fprintf('error');
                %end
                if Bi(j,i,k)==1 || K(j,i)==k
                    E(j,i,k) = m(k) * sqrt((i-init_grid(k,1))^2+(j-init_grid(k,2))^2);
                else
                    E(j,i,k) = 999999;
                end
            end
            [~,index] = min(E(j,i,:));
            
            % check connectivity
            isconnect = 1;
            E0 = sqrt((i-init_grid(index,1))^2+(j-init_grid(index,2))^2);
            for k=1:robot_num
               % check high risk robot
               Et = sqrt((i-init_grid(k,1))^2+(j-init_grid(k,2))^2);
               if E0 - Et > 1.2*sqrt((init_grid(index,1)-init_grid(k,1))^2+(init_grid(index,2)-init_grid(k,2))^2)/sqrt(5) % user-defined hyperbola 
                   isconnect = 0;
               end
            end
            
            % finish loop
            if isconnect == 1
                K(j,i) = index;
                S(index) = S(index)+1;
            else
                S(K(j,i)) = S(K(j,i)) + 1;
            end
        end
    end
    
    %% update m
    %dm = zeros(1,robot_num);
    dm = S - (F-ob)/robot_num;
    for k=1:robot_num
       if dm(k) > 2 || dm(k) < -2
          m(k) = m(k)+0.002*dm(k);
       else
           count = count + 1;
       end
    end
    
    if count == robot_num
        stop = 1;
    end
    %S
    %m
    K;
    %dm
    %count
    Area_rst = K;
    figure(2);
    handle2 = display_area(Area_rst,robot_num,2);
end
m
S
%K = K/10;
%colormap([0 0 0;rand(robot_num,3)]);
%pcolor(1:L,1:W,K);


end

