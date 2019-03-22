function [ Area_rst ] = divide_area( Core_flag, init_grid, robot_num )
% Area_flag:L*W (L:length of SOI, W:width of SOI, Area_flag(i,j)=1 means free space/0 means obstacles)
% init_pos:robot_num*2
% Area_rst:L*W*robot_num (Area_rst(:,:,i) means the area for robot i)

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
D = ones(1,robot_num)*(-1); %keep track of connectivity radius
D_vec = zeros(robot_num,2);
C = ones(W,L,robot_num); %modified matrix
penality_theshold = 3;
%S = zeros(1,robot_num);

%display_area(Area_flag,1);

stop = 0;
isfirst = 1;
while stop==0
    S = zeros(1,robot_num);
    ob = 0;
    count = 0;
    %% check every grid
    for i=1:L
        for j=1:W
            if Core_flag(j,i)==-1
                ob = ob + 1;
                continue;
            end

            for k=1:robot_num
                %if m(k)<0
                %    fprintf('error');
                %end
                E(j,i,k) = C(j,i,k) * m(k) * sqrt((i-init_grid(k,1))^2+(j-init_grid(k,2))^2);
            end
            [~,index] = min(E(j,i,:));
            K(j,i) = index;
            S(index) = S(index)+1;
            rst_t = sqrt((i-init_grid(index,1))^2+(j-init_grid(index,2))^2);
            if isfirst == 1
                if D(index) < rst_t
                    D(index) = rst_t;
                    D_vec(index,:) = [i-init_grid(index,1),j-init_grid(index,2)];
                end
            else
                c = D_vec(index,:)*[i-init_grid(index,1);j-init_grid(index,2)];
                d = rst_t - c/rst_t;
                if d > penality_theshold
                    C(j,i,index) = C(j,i,index) * d;
                elseif d > 0
                    D(index) = rst_t;
                end
            end
        end
    end
    isfirst = 0;
   
    %% update m
    %dm = zeros(1,robot_num);
    dm = S - (F-ob)/robot_num;
    for k=1:robot_num
       if dm(k) > 1.8 || dm(k) < -1.8
          m(k) = m(k) + 0.002 * dm(k);
       else
           count = count + 1;
       end
    end
    %for i=1:L
    %    for j=1:W
    %        if Area_src(j,i)~=1
    %            continue;
    %        end
    %        if dm() <= 1 || dm >= -1
    %            m(j,i) = m(j,i) - 0.2 * dm();
    %            count = count + 1;
    %        end
    %    end
    %end
    if count == robot_num
        stop = 1;
    end
    S
    %m
    %dm
    %count
    Area_rst = K;
    figure(2);
    handle2 = display_area(Area_rst,robot_num,2);
end
%m
%S
%K = K/10;
%colormap([0 0 0;rand(robot_num,3)]);
%pcolor(1:L,1:W,K);


end

