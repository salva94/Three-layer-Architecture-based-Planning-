function showNetRoad()
global x_index;
global y_index;
global self_id_index;
global son_number_index;
global parent_id_index;
global g_index;
global h_index;
global f_index;
x_index = 1; y_index = 2; self_id_index = 3; 
parent_id_index = 4; 
g_index = 5; h_index = 6;
f_index = 7;%cost value
son_number_index = 10;%number of child node
kx=4.016995297405768e+05;
px=390;
ky=5.313104448721175e+05;
py=-90;
minX = 1.162859469000000e+02;
minY = 39.850357099999997;
img=imread('img.png');
[height width  m]=size(img);
imshow(img*2)
format long
file_name='map_LZY_temp21.csv';
  point_inf=[];
   point_inf=dlmread(file_name);
    hold on
  axis on, axis equal
   [n m]=size(point_inf);
   for i=1:n
       x=point_inf(i,x_index);
       y=point_inf(i,y_index);
       x=(x-minX)*kx+px;
       y=height-(y-minY)*ky+py;
       plot(x,y,'ro','MarkerSize',9,'MarkerFaceColor',[1,0,0]);
       disp_d=sprintf('%d',point_inf(i,self_id_index));
       text(x,y,disp_d,'color','r','FontSize',25);
   end
   for i=1:n
       num=point_inf(i,son_number_index);
       x1=point_inf(i,x_index);
       y1=point_inf(i,y_index);
       x1=(x1-minX)*kx+px;
       y1=height-(y1-minY)*ky+py;
       for j=1:num
           index=point_inf(i,son_number_index+j);
           id=find(point_inf(:,self_id_index)==index);
           x2=point_inf(id,x_index);
           y2=point_inf(id,y_index);
           x2=(x2-minX)*kx+px;
           y2=height-(y2-minY)*ky+py;
           p1=[x1 x2];
           p2=[y1 y2];
           plot(p1,p2,'b-','LineWidth',1.4);
       end
   end
end