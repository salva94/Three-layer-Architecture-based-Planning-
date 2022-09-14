%%
%时间：20200430
%作者：sek
%功能：为了写高分项目，把multiRobots_hunts.m生成的三条路径投影到卫星地图上。
close all,clc,clear all
  global x_index;
    global y_index;
    global self_id_index;
    global son_number_index;
    global parent_id_index;
    global g_index;
    global h_index;
    global f_index;
    global data;
    x_index = 1; y_index = 2; self_id_index = 3; 
    parent_id_index = 4; 
    g_index = 5; h_index = 6;
    f_index = 7;%cost value
    son_number_index = 10;%number of child node
%file_nameNum=dir('result/*.csv');
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
  
  filename='output4.csv';
  data1=dlmread(filename);
  filename='output5.csv';
  data2=dlmread(filename);
  filename='output6.csv';
  data3=dlmread(filename);
    
 file_name='map_LZY_temp2.csv';%输入路网信息
  point_inf=dlmread(file_name);

    hold on
  axis on, axis equal
   [n m]=size(point_inf);
   for i=1:n
       x=point_inf(i,x_index);
       y=point_inf(i,y_index);
       x=(x-minX)*kx+px;
       y=height-(y-minY)*ky+py;
       plot(x,y,'o');
       disp_d=sprintf('%d',point_inf(i,self_id_index));
       text(x,y,disp_d,'FontSize',18);
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
           plot(p1,p2,'b-');
       end
   end
 %%

 hold on,axis on, axis equal

     line=data1;
     x=line(:,3);
     y=line(:,2);
     x=(x-minX)*kx+px;
     y=height-(y-minY)*ky+py;
     plot(x,y,'r-o');
     
       line=data2;
     x=line(:,3);
     y=line(:,2);
     x=(x-minX)*kx+px;
     y=height-(y-minY)*ky+py;
     plot(x,y,'g-o');
     
       line=data3;
     x=line(:,3);
     y=line(:,2);
     x=(x-minX)*kx+px;
     y=height-(y-minY)*ky+py;
     plot(x,y,'b-o');

    start_id_v1=10;
   start_id_v2=1;
  start_id_v3=17;
  start_id1=find(start_id_v1==point_inf(:,self_id_index));
   x=point_inf(start_id1,x_index);
   y=point_inf(start_id1,y_index);
   x=(x-minX)*kx+px;
   y=height-(y-minY)*ky+py;
   plot(x,y,'rp','LineWidth',5);
  % text(point_inf(index,x_index),point_inf(index,y_index),'    1号无人平台')
   start_id2=find(start_id_v2==point_inf(:,self_id_index));
    x=point_inf(start_id2,x_index);
   y=point_inf(start_id2,y_index);
   x=(x-minX)*kx+px;
   y=height-(y-minY)*ky+py;
   plot(x,y,'gp','LineWidth',5);
  
   start_id3=find(start_id_v3==point_inf(:,self_id_index));
    x=point_inf(start_id3,x_index);
   y=point_inf(start_id3,y_index);
   x=(x-minX)*kx+px;
   y=height-(y-minY)*ky+py;
   plot(x,y,'bp','LineWidth',5);
  
   end_id_v1=12; 
   end_id_v2=11;
   end_id_v3=7;
   end_id1=find(end_id_v1==point_inf(:,self_id_index));
    x=point_inf(end_id1,x_index);
   y=point_inf(end_id1,y_index);
   x=(x-minX)*kx+px;
   y=height-(y-minY)*ky+py;
   plot(x,y,'ks','LineWidth',5);
    
   end_id2=find(end_id_v2==point_inf(:,self_id_index));
    x=point_inf(end_id2,x_index);
   y=point_inf(end_id2,y_index);
   x=(x-minX)*kx+px;
   y=height-(y-minY)*ky+py;
   plot(x,y,'ks','LineWidth',5);
   
    end_id3=find(end_id_v3==point_inf(:,self_id_index));
    x=point_inf(end_id3,x_index);
   y=point_inf(end_id3,y_index);
   x=(x-minX)*kx+px;
   y=height-(y-minY)*ky+py;
   plot(x,y,'ks','LineWidth',5);
 %end