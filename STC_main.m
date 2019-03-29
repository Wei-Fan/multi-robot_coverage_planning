Test for STC path generation

preparation
LENGTH = 20;

rng(30,'twister');
A_core = unifrnd(0,1,[LENGTH/2,LENGTH/2]);
den = 0.05;
A_core(A_core>den) = 1;% freee space
A_core(A_core<den) = 0;% obstacle

B = A_core;
B(end+1,end+1) = 0;

colormap([0 0 0;1 1 1]);  % color
pcolor(0.5:size(A_core,2)+0.5,0.5:size(A_core,1)+0.5,B); % grid color
set(gca,'XTick',1:size(A_core,2),'YTIck',1:size(A_core,1)); 
axis image xy;  % axis image is the same as axis equal except that the plot box fits tightly around the data.


kn = ones(2);
A = kron(A_core,kn);

%B = A;
%B(end+1,end+1) = 0;
%
%colormap([0 0 0;1 1 1]);  % color
%pcolor(0.5:size(A,2)+0.5,0.5:size(A,1)+0.5,B); % grid color
%set(gca,'XTick',1:size(A,2),'YTIck',1:size(A,1)); 
%axis image xy;  % axis image is the same as axis equal except that the plot box fits tightly around the data.



Generate Spanning tree
start_p = round(unifrnd(1,LENGTH/2,[2,1]))
if A_core(start_p(2),start_p(1))==1
    'good'
end
K = zeros(LENGTH/2); % Input
for j=1:LENGTH/2
    for i=1:LENGTH/2
        if A_core(j,i)==1
            K(j,i) = 1;
        end
    end
end
dir = [0 1 0 -1;1 0 -1 0];
Total_num = sum(sum(K))
T = zeros(LENGTH/2);
num = 1;
T(start_p(2),start_p(1)) = num;
curr_p = zeros(2,1);
for d=dir
    tem_p = start_p + d;
    if tem_p(1)<=0||tem_p(1)>LENGTH/2||tem_p(2)<=0||tem_p(2)>LENGTH/2
        continue;
    end
    if K(tem_p(2),tem_p(1))==1
        line([tem_p(1),start_p(1)],[tem_p(2),start_p(2)],'Color','r','LineWidth',2)
        curr_p = tem_p;
        num = num + 1;
        T(tem_p(2),tem_p(1)) = num;
        break;
    end
end

while num~=Total_num
    %num
    % move to free space
    move = 0;
    for d=dir
        tem_p = curr_p + d;
        if tem_p(1)<=0||tem_p(1)>LENGTH/2||tem_p(2)<=0||tem_p(2)>LENGTH/2
            continue;
        end
        if K(tem_p(2),tem_p(1))==1 && T(tem_p(2),tem_p(1))==0
            line([tem_p(1),curr_p(1)],[tem_p(2),curr_p(2)],'Color','r','LineWidth',2)
            curr_p = tem_p;
            num = num + 1;
            T(tem_p(2),tem_p(1)) = num;
            move = 1;
            break;
        end
    end
    % back to an old cell
    if move==1
        continue;
    end
    for d=dir
        tem_p = curr_p + d;
        if tem_p(1)<=0||tem_p(1)>LENGTH/2||tem_p(2)<=0||tem_p(2)>LENGTH/2
            continue;
        end
        if K(tem_p(2),tem_p(1))==1 && T(tem_p(2),tem_p(1))==T(curr_p(2),curr_p(1))-1
            curr_p = tem_p;
            break;
        end
    end
end




