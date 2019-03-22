function [ handler] = display_area( Area, object_num, figure_num )
% Area:L*W (L:length of SOI, W:width of SOI, Area(i,j)=1 means free space/0 means obstacles)
%% draw the grid map
%figure(figure_num);
B = Area;
B(end+1,end+1) = 0;
if object_num == 1
    colormap([0 0 0;1 1 1]);  % color
else
    colormap([0 0 0;unifrnd(0,1,[object_num,3])]);
end
pcolor(0.5:size(Area,2)+0.5,0.5:size(Area,1)+0.5,B); % grid color
set(gca,'XTick',1:size(Area,2),'YTIck',1:size(Area,1)); % set coordinate
%axis image xy;
handler = gcf;
name = ['subarea_',num2str(figure_num)];
set(handler,'Name',name);

end

