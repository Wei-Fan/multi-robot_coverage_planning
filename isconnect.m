function [ bool ] = isconnect( j0,i0,K,init_robot )
% j-W,i-L
% K=1 represents assigned,0 represents unassigned
% remeber the iteration journey and increase the cost of turning back
pace = 0;
coordinate = [0 1 0 -1;-1 0 1 0];% 1-up 2-right 3-down 4-left
robot = zeros(2,1);
robot(1) = init_robot(2);
robot(2) = init_robot(1);
curr_p = [j0;i0];

stop = 0;
count = 0;
while stop==0
   %% see if the condition were met
   curr_p;
   dir = robot - curr_p;
   if dir(1)==0 && dir(2)==0
       bool = 1;
       stop = 1;
       continue;
   end
   dir = dir/sqrt(sum(dir.^2));
   
   dir_cost = sqrt(sum((coordinate - dir).^2));
   if pace~=0
      dir_cost(pace) = dir_cost(pace) + 2; 
   end
   [~,dir_i] = sort(dir_cost);
   sorted_cd = zeros(2,4);
   for i=1:4
       sorted_cd(:,i) = coordinate(:,dir_i(i));
   end
   %dir_cost
   %sorted_cd
   %% choose the nearest approachable way
   move = 0;
   for sd=sorted_cd
       next_p = zeros(2,1);
       next_p = curr_p + sd;
       if next_p(1)==0||next_p(1)==size(K,1)+1||next_p(2)==0||next_p(2)==size(K,2)+1
           continue;
       end
       if K(next_p(1),next_p(2))==1
           curr_p = next_p;
           move = 1;
           if sd(1)==1
               pace = 4;
           elseif sd(1)==-1
               pace = 2;
           elseif sd(2)==-1
               pace = 3;
           else
               pace = 1;
           end
           break;
       end
   end
   if move==0
       bool = 0;
       stop = 1;
   else
       count = count+1;
       if count==size(K,1)*size(K,2)
           bool = 0;
           stop = 1;
       end
   end
end

end
