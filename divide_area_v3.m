function [ Area_rst ] = divide_area_v3( Core_flag, init_grid, robot_num )
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
coef = ones(1,robot_num)*0.002;

%S = zeros(1,robot_num);

%display_area(Area_flag,1);

stop = 0;
isfirst = 1;
while stop==0
    S = zeros(1,robot_num);
    K0 = zeros(W,L); %temporary assignment matrix
    acc = zeros(1,robot_num);
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
                E(j,i,k) = m(k) * sqrt((i-init_grid(k,1))^2+(j-init_grid(k,2))^2);
            end
            [~,index] = min(E(j,i,:));
            if isfirst == 1
                K0(j,i) = index;
            else
                go = 0;
                if j>=2
                    if K0(j-1,i) == index
                        go = go+1;
                    end
                end
                if i>=2
                    if K0(j,i-1) == index
                        go = go+1;
                    end
                end 
                if j<=W-1
                    if K0(j+1,i) == index
                        go = go+1;
                    end
                end
                if i<=L-1
                    if K0(j,i+1) == index
                        go = go+1;
                    end
                end
                if go >= 2
                    K0(j,i) = index;
                else
                    K0(j,i) = K(j,i);
                    acc(index) = acc(index) + 1;
                    index = K(j,i);
                    
                end
            end
            S(index) = S(index)+1;
        end
    end
    isfirst = 0;
    K = K0;
   
    %% update m
    %dm = zeros(1,robot_num);
    dm = S - (F-ob)/robot_num;
    for k=1:robot_num
       if dm(k) > 2 || dm(k) < -2
          coef(k) = coef(k) + acc(k)*0.0002;
          m(k) = m(k) + coef(k) * dm(k);
       else
           count = count + 1;
       end
    end
 
    
    if count == robot_num
        stop = 1;
    end
    S
    m
    coef
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

