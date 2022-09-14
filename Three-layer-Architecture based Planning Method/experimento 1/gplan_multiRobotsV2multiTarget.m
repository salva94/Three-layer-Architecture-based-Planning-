function gplan_multiRobotsV2multiTarget()
clear all;clc;close all
%多机器人搜寻多目标算法
%鼠标左键在地图上点击选择机器人位置，可以多选，按右健结束，然后鼠标左键在地图上点击选择目标位置，数量与机器人一致，按右健结束
%思路，这是个多机器人多目标的分配问题，首先，利用A星算法，得到每个机器人到每个目标之间的最优
%路径，计算出其长度，然后利用匈牙利算法对N*N之间的目标分配问题求最优解，得到一组一一对应的关系，
%从而给出多机器人去多目标的路径
%% index defintion
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

file_name='map_LZY_temp2.csv';
%file_name='map_temp.csv';
%file_name='map_temp_format88.csv';
point_inf=dlmread(file_name);

Show_relationship(point_inf);
%% set start point and target point
   disp('选择机器人位置，按右键结束!');
   [start_id ]=FindNode(point_inf);
   start_id_v=point_inf(start_id,self_id_index);
   hold on
   plot(point_inf(start_id,x_index),point_inf(start_id,y_index),'gp');
   text(point_inf(start_id,x_index)+0.5,point_inf(start_id,y_index),'S')
   hold off
   [end_id ]=FindNode(point_inf);
   end_id_v=point_inf(end_id,self_id_index);
   hold on
   plot(point_inf(end_id,x_index),point_inf(end_id,y_index),'rp');
   text(point_inf(end_id,x_index)+0.5,point_inf(end_id,y_index),'T')
   
%% 把所有可能的解都给出来
   robot_number=size(start_id,2);
   target_number=size(end_id,2);
   total_result_list=zeros(target_number*robot_number,30);
   result_list_length=zeros(target_number,robot_number);
   tempk=1;
   for i=1:target_number
       for j=1:robot_number
           [result_list flag]=ResearchPath(start_id(j),end_id(i),point_inf);
           if flag==1
               total_result_list(tempk,1:size(result_list,2))=result_list;
               result_list_length(j,i)=dis_path(result_list,point_inf );
           end
           tempk=tempk+1;
       end
   end
   result_list_length;%这是一个代价矩阵，里面表示每个机器人到每个目标之间的代价值
   %% 利用匈牙利算法进行分配
   [C,T]=hungarian(result_list_length);
   
   for i=1:target_number
      list=[];
     % for j=1:size(total_result_list,2)
%            if total_result_list((i-1)*target_number+C(i),j)>0
%                list=[list total_result_list((i-1)*target_number+C(i),j)];
%            end
           list_index=find(total_result_list((i-1)*target_number+C(i),:)~=0);
           list=total_result_list((i-1)*target_number+C(i),list_index);
    %   end
      x=point_inf(list,x_index);
      y=point_inf(list,y_index);
      disp_d=sprintf(' %d  ',i); 
      text(x(1)-0.5,y(1)-0.5,disp_d)
      text(x(end)-0.5,y(end)-0.5,disp_d)
      switch i
          case {1}
            plot(x,y,'r-o','LineWidth',2)
          case {2}
            plot(x,y,'g-o','LineWidth',2)  
          case {3}
             plot(x,y,'black-o','LineWidth',2)  
          case {4}
             plot(x,y,'c-o','LineWidth',2) 
          case {5}
             plot(x,y,'m-o','LineWidth',2) 
          case {6}
             plot(x,y,'y-o','LineWidth',2) 
      end
      index=point_inf(list,self_id_index);
      generateLine(index',i+3);
   end
  
   disp_d=sprintf('target number：%d robots number: %d  ',target_number,robot_number);
   disp(disp_d)
       

end
function dis=dis_twoNodes(ax,ay,bx,by)
    dis=sqrt((ax-bx)^2+(ay-by)^2);
end
function dis=dis_path(list,point_inf )
global x_index;
global y_index;
    dis=0;
    for i=1:size(list,2)-1
        dis=dis+dis_twoNodes(point_inf(list(i),x_index),point_inf(list(i),y_index),point_inf(list(i+1),x_index),point_inf(list(i+1),y_index));
    end
end

function Show_relationship(point_inf)
global x_index;
global y_index;
global son_number_index;
global self_id_index;
 hold on
  axis on, axis equal
   [n m]=size(point_inf);
   for i=1:n
       x=point_inf(i,x_index);
       y=point_inf(i,y_index);
       plot(x,y,'o');
       disp_d=sprintf('%d',point_inf(i,self_id_index));
        text(x,y,disp_d,'FontSize',18);
   end
   for i=1:n
       num=point_inf(i,son_number_index);
       x1=point_inf(i,x_index);
       y1=point_inf(i,y_index);
       for j=1:num
           index=point_inf(i,son_number_index+j);
           id=find(point_inf(:,self_id_index)==index);
           x2=point_inf(id,x_index);
           y2=point_inf(id,y_index);
           p1=[x1 x2];
           p2=[y1 y2];
           plot(p1,p2,'b-');
       end
   end
end

function [result_list flag]=ResearchPath(start_id,end_id,point_inf)
global x_index;
global y_index;
global self_id_index;
global son_number_index;
global parent_id_index;
global g_index;
global h_index;
global f_index;
 result_list=[];
 open_list=[];
 start_node=point_inf(start_id,:);
 end_node=point_inf(end_id,:);

 g_value=0;
 h_value=sqrt((start_node(x_index)-end_node(x_index))^2+(start_node(y_index)-end_node(y_index))^2);
 f_value=g_value+h_value;
 point_inf(start_id,h_index)=h_value;
 point_inf(start_id,g_index)=g_value;
 point_inf(start_id,f_index)=f_value;
 point_inf(start_id,parent_id_index)= point_inf(start_id,self_id_index);
 open_list(1) = start_id;
 closed_list = [];
 while(1)
    % #1
    if(isempty(open_list))
        disp('fail!!!')
        flag =0;
        break;
    end  
    [min_cost, min_cost_index] = min(point_inf(open_list,f_index));
    current_id = open_list(min_cost_index);
    closed_list =[closed_list current_id];
    open_list(min_cost_index) = [];
    
    if(is_end(current_id, end_id,point_inf))
        result_list=current_id;
        temp_id=current_id;
        flag =1;
        while(1)
            current_id_v=point_inf(temp_id,parent_id_index);
            current_id =find(current_id_v==point_inf(:,self_id_index));
            if temp_id==current_id
                break;
            end
            result_list=[current_id result_list];
            temp_id=current_id;
        end
        break;
    end
    %next = node_expand(current,point_inf);
    child_node_number=point_inf(current_id,son_number_index);
    current_node=point_inf(current_id,:);
    for i=1:child_node_number
        temp_id=point_inf(current_id,son_number_index+i);
        next_id=find(temp_id==point_inf(:,self_id_index));
        next_node=point_inf(next_id,:);
        
        g_value=sqrt((current_node(x_index)-next_node(x_index))^2+(current_node(y_index)-next_node(y_index))^2)+current_node(g_index);
        h_value=sqrt((end_node(x_index)-next_node(x_index))^2+(end_node(y_index)-next_node(y_index))^2);
        f_value=g_value+h_value;
        if next_node(f_index)>f_value
            point_inf(next_id,f_index)=f_value;
            point_inf(next_id,g_index)=g_value;
            point_inf(next_id,h_index)=h_value;
            point_inf(next_id,parent_id_index)=current_node(self_id_index);
            open_list=[open_list next_id];
        end
    end
 end
end
function [p1]=FindNode(point_inf)

    global x_index;
    global y_index;
    hold on
    buttom=0;
    p1=[];
    while(1)
       [x y buttom]=ginput(1);
       if buttom==3
           break;
       end
       total_num=size(point_inf,1);
       dis1=[];
       for i=1:total_num
           x1=point_inf(i,x_index);
           y1=point_inf(i,y_index);
           dis=sqrt((x1-x(1))^2+(y1-y(1))^2);
           dis1=[dis1 dis];
       end
       [data p]=min(dis1);
       plot(point_inf(p,x_index),point_inf(p,y_index),'ro')
       p1=[p1 p];
    end
    hold off
end
function [p1 p2]=FindTwoNode(point_inf)

    global x_index;
    global y_index;
    buttom=0;
   [x y buttom]=ginput(2);
   total_num=size(point_inf,1);
   dis1=[];
   dis2=[];
   for i=1:total_num
       x1=point_inf(i,x_index);
       y1=point_inf(i,y_index);
       dis=sqrt((x1-x(1))^2+(y1-y(1))^2);
       dis1=[dis1 dis];
       dis=sqrt((x1-x(2))^2+(y1-y(2))^2);
       dis2=[dis2 dis];
   end
   [data p1]=min(dis1);
   [data p2]=min(dis2);
   
end
function ret = is_end(point, end_point,point_inf)

    %% index defintion
    global x_index;
    global y_index;
    global self_id_index;
    global son_number_index;
    global parent_id_index;
    global g_index;
    global h_index;
    global f_index;
    threshod=sqrt((point_inf(1,x_index) - point_inf(2,x_index))^2 + (point_inf(1,y_index) - point_inf(2,y_index))^2);
    
    position_error = sqrt((point_inf(point,x_index) - point_inf(end_point,x_index))^2 + (point_inf(point,y_index) - point_inf(end_point,y_index))^2);
    ret = 0;
    if(position_error < threshod*0.001 )
        ret = 1;
    end
end