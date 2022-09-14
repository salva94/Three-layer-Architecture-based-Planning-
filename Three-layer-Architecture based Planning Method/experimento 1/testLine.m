function testLine()
%%
%时间：20200326
%作者：sek
%功能：测试choose_line()生成的各种路段是否正确，首先需要把路段文件输入

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
file_nameNum=dir('result/*.csv');

format long
data=[];
for i=1:size(file_nameNum)%选择路段名字并且输入
  
    filename=['result/',file_nameNum(i).name];
    td=dlmread(filename);
    data=[data; td];
end
 file_name='map_LZY_temp21.csv';%输入路网信息
  point_inf=dlmread(file_name);
 %%
 position=find(data(:,4)==12345.0);%从路段里寻找校验码，找出所有的路段头
figure,hold on,axis on, axis equal
 for i=1:size(position,1)
     num=position(i);
     tn=data(num,3);
     startP=data(num,1);
     endP=data(num,2);
     line=data(num+1:num+tn,:);
     temp=mod(i,5);
     if temp==0
        plot(line(:,3),line(:,2),'r-o');
     elseif temp==1
        plot(line(:,3),line(:,2),'g-o');
     elseif temp==2
        plot(line(:,3),line(:,2),'b-o');
     elseif temp==3
        plot(line(:,3),line(:,2),'c-o');
     elseif temp==4
        plot(line(:,3),line(:,2),'m-o');
     end
     index=find(startP==point_inf(:,self_id_index));
     x=point_inf(index,x_index);
     y=point_inf(index,y_index);
      plot(x,y,'b*')
      disp_d=sprintf('%d',point_inf(index,self_id_index));
      text(x,y,disp_d,'FontSize',18);
      index=find(endP==point_inf(:,self_id_index));
      x=point_inf(index,x_index);
      y=point_inf(index,y_index);
      plot(x,y,'black*')
      disp_d=sprintf('%d',point_inf(index,self_id_index));
      text(x,y,disp_d,'FontSize',18);
 end
%   msgbox('下面需要手工调整');
%   %用于手工调整某一段的偏移量
%   a=1;
%   if a==1
%     startP=2.0;
%      endP=18.0;
%      valueX=0.5;
%      valueY=0;
%      moveLine(startP,endP,valueX,valueY);
%      
%      startP=14.0;
%      endP=13.0;
%      valueX=-0.8;
%      valueY=0;
%      moveLine(startP,endP,valueX,valueY);
%   end
%   file_name='final_data.csv';
%   final_data=data;
%   dlmwrite(file_name,final_data,'precision',18);
end

function moveLine(startP,endP,valueX,valueY)
   global data;
    position=find(((data(:,1)==startP )  & (data(:,2)==endP ))  | ((data(:,2)==startP )  & (data(:,1)==endP )) );
     indexP=data(position,:);
     final_d=data(position+1:position+indexP(3),:);
     x=final_d(:,3);
     y=final_d(:,2);
    x1=[0; x];    y1=[0; y];    x2=[x; 0];    y2=[y ;0];    dx=x2-x1;    dy=y2-y1;
    dx=dx(2:end-1);    dy=dy(2:end-1);
    dis=sqrt(dy.*dy+dx.*dx);
    maxV=max(dis);
    minV=min(dis);
    averV=sum(dis)/size(dis,1);
    x(2:end-1)=x(2:end-1)+averV*valueX;%这个地方是用来调整偏移量的，同理可以减去y的
    y(2:end-1)=y(2:end-1)+averV*valueY;
     plot(x,y,'g-*'); 
     final_d(:,3)=x;
     final_d(:,2)=y;   
    data(position+1:position+indexP(3),:)=final_d;
     disp_d=sprintf('%d',indexP(1));
     x=final_d(1,3);
     y=final_d(1,2);
end