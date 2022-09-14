function gplan_multiRobotsV2()
clear all;clc;close all
%多机器人单目标围剿算法，主要思路是目标有几个岔路口，就搜寻几条道路
%鼠标左键在地图上点击选择机器人位置，选择目标位置，自动给出需要的路径和个数
%思路：输入是目标位置和起始点位置，根据目标点位置有多少个岔路N，得到需要多少个机器人
%去堵路，利用A星搜索算法先得到最优的一条路径，然后断开该路径到目标点的最后一个节点，
%重新进行A星搜索，得到当前条件下的最优路径，不断的迭代，直到找到N条不同岔口的路径，分配给
%不同的机器人
%2020年3月26号版本：选择一条路，调用generateLine()生成真实的路径
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
%file_name='map_LZY_temp2.csv';
point_inf=dlmread(file_name);

Show_relationship(point_inf);
%% set start point and target point
   disp('choose start point and target point!');
   [start_id ]=FindNode(point_inf);
   start_id_v=point_inf(start_id,self_id_index);
   plot(point_inf(start_id,x_index),point_inf(start_id,y_index),'gp');
   text(point_inf(start_id,x_index)+0.5,point_inf(start_id,y_index),'S')
   [end_id ]=FindNode(point_inf);
   end_id_v=point_inf(end_id,self_id_index);
   plot(point_inf(end_id,x_index),point_inf(end_id,y_index),'rp');
   text(point_inf(end_id,x_index)+0.5,point_inf(end_id,y_index),'T')
%% make sure how many robots are need
   robot_number=0;
   for i=1:point_inf(end_id,son_number_index)
       if point_inf(end_id,son_number_index+i)~=point_inf(end_id,self_id_index)
           robot_number=robot_number+1;
       end
   end
   disp_d=sprintf('need %d robots ',robot_number);
   disp(disp_d)
       
 %% multi roads research algorithm 
 hold on 
 robot_number=1;%强行只要求一条路径
 singla_result=[];%需要输出的路径序列结果
 while(robot_number>0)
      [result_list flag]=ResearchPath(start_id,end_id,point_inf);
      %singla_result=result_list;
      if flag==1
           dis=dis_path(result_list,point_inf );
           disp_d=sprintf('path of robots%d : length of path: %5.5d',robot_number,dis);
           disp(disp_d)
           t=[];
           for i=1:size(result_list,2)
               t=[t point_inf(result_list(i),self_id_index)];
           end
           disp(t)
           singla_result=t;
      end
      x=point_inf(result_list,x_index);
      y=point_inf(result_list,y_index);
      switch robot_number
          case {1}
            plot(x,y,'r-o')
          case {2}
            plot(x,y,'g-o')  
          case {3}
             plot(x,y,'black-o')  
          case {4}
             plot(x,y,'c-o') 
      end
    %% cancel relationship
    if flag==1
         p1=result_list(end-1);
         p2=end_id;
         for i=1:point_inf(p1,son_number_index)
              if point_inf(p2,self_id_index)==point_inf(p1,son_number_index+i)
                   point_inf(p1,son_number_index+i)=point_inf(p1,self_id_index);
              end
         end
         for i=1:point_inf(p2,son_number_index)
              if point_inf(p1,self_id_index)==point_inf(p2,son_number_index+i)
                    point_inf(p2,son_number_index+i)=point_inf(p2,self_id_index);
              end
         end
    end
     robot_number=robot_number-1;
     %% clear parameter
     for i=1:size(point_inf,1)
         point_inf(i,parent_id_index)=0;
         point_inf(i,g_index)=0;
         point_inf(i,h_index)=0;
         point_inf(i,f_index)=1000;
     end
  end 
generateLine(singla_result);
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
    buttom=0;
   [x y buttom]=ginput(1);
   total_num=size(point_inf,1);
   dis1=[];

   for i=1:total_num
       x1=point_inf(i,x_index);
       y1=point_inf(i,y_index);
       dis=sqrt((x1-x(1))^2+(y1-y(1))^2);
       dis1=[dis1 dis];
   end
   [data p1]=min(dis1);
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