function [ output_args ] = divide_area_v2( Core_flag, init_grid, robot_num )
% Area_flag:L*W (L:length of SOI, W:width of SOI, Area_flag(i,j)=1 means free space/0 means obstacles)
% init_pos:robot_num*2
% Area_rst:L*W*robot_num (Area_rst(:,:,i) means the area for robot i)
%i-x,j-y

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
K = zeros(W,L); %assignment matrix
C = ones(W,L,robot_num); %modified matrix
%penality_theshold = 3;
%S = zeros(1,robot_num);

%display_area(Area_flag,1);

stop = 0;
isfirst = 1;
while stop==0
    S = zeros(1,robot_num); %number of grids for each region
    D = ones(1,robot_num)*(-1); %keep track of largest grid distance
    D_vec = zeros(robot_num,2); %vector of the D
    ob = 0;
    count = 0;
    R_vec = zeros(robot_num,2);%region vector
    R_svec = zeros(robot_num,2);%region square vector
    %% check every grid
    % assignment each grid and calculate region parameters
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
            R_vec(index,:) = R_vec(index,:) + [j,i] - init_grid(index,:);
            R_svec(index,:) = R_svec(index,:) + ([j,i] - init_grid(index,:)).^2;
            
            rst_t = sqrt((i-init_grid(index,1))^2+(j-init_grid(index,2))^2);
            if D(index) < rst_t
                D(index) = rst_t;
                D_vec(index,:) = [i-init_grid(index,1),j-init_grid(index,2)];
            end
        end
    end
    % finish grid assignment
    R_vec = R_vec ./ [S; S]';
    R_sigma = sqrt(R_svec ./ [S; S]')

    
    %% update modified 
    h = @(v,ind) exp(-sum(((v - R_vec(ind,:))./R_sigma(ind,:)).^2) / 2); % Guass
    q1 = 2; q2 = 1.5;
    g = @(v,ind) q1/(sum(v - init_grid(ind,:)))^0.4 + q2/(sum(v - init_grid(ind,:) - R_vec(ind,:)))^0.4; % charge like field
    % update through each grid
    for i=1:L
        for j=1:W
            if Core_flag(j,i)==-1
                ob = ob + 1;
                continue;
            end
            index = K(j,i);
            
            % update modified matrix -- input:
            % init_grid,(x,y),D,D_vec,R_vec,R_sigma
            % output: C
            for ind = 1:robot_num
                the = g(D_vec(ind,:)*q2/(q1+q2)+init_grid(ind,:),ind);
                z1 = h([j,i],ind);
                z2 = g([j,i],ind);
                if z2 > the
                    z2 = 1;
                else
                    z2 = z2 / the;
                end
                z = z1 * z2;
                if z > 0.25
                    z = 1;
                else
                    z = z / 0.25;
                end
                C(j,i,ind) = z;
            end
        end
    end
   
    %% update m
    %dm = zeros(1,robot_num);
    dm = S - (F-ob)/robot_num;
    for k=1:robot_num
       if dm(k) > 2 || dm(k) < -2
          m(k) = m(k) + 0.002 * dm(k);
       else
           count = count + 1;
       end
    end
    
    %% stop condition
    if count == robot_num
        stop = 1;
    end
    S
    %C(:,:,1)
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


