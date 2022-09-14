function GA_multi2multi_main()
%时间：20210202
%作者:sek
%用途：用于写论文，基于级联遗传算法的多无人车多目标运输方法
%思路：多无人车根据最大值最小原则进行任务分配
    close all,clc,clear all
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
    im=imread('map_final.png');
    im=rgb2gray(im);
    [img_width img_height]=size(im);
    fig_map=figure,imshow(im)
    file_name='map3.csv';
    point_inf=dlmread(file_name);
    point_inf(:,7)=10000;
    Show_relationship(point_inf);
    target_p=[85 85 60 48 101 54 32 24 67 64 35 109 76 5 14 74 117 122 124 8];%目标位置，第一个位置没用
    xy=point_inf(target_p,1:2);
    start_p=[131 133 138 139];%无人车位置
    startP=point_inf(start_p,1:2);
    hold on
    plot(point_inf(:,1),point_inf(:,2),'ro');
    plot(xy(:,1)-1,xy(:,2),'gp','LineWidth',5);
    plot(startP(:,1),startP(:,2),'rp','LineWidth',5);

    %%
    Vari_num=8;%变异分支
    salesmen = size(startP,1);%旅行商个数
    N = size(xy,1);%城市个数
    dmat=zeros(salesmen,N,N);
    for i=1:salesmen%生成路径距离矩阵
        target_p(1)=start_p(i);%目标序列的第一个位置是用来给出发点的
        for j=1:N
            for k=1:N
                start_id= target_p(j);
                end_id=target_p(k);
                [result_list flag]=ResearchPath(start_id,end_id,point_inf);
                 result_list_length=dis_path(result_list,point_inf );
                 dmat(i,j,k) = result_list_length;
            end
        end
    end
    min_tour = 1;%每个人最小的长度
    pop_size = Vari_num*800;%人口数量，需要是变异种类的倍数
    num_iter = 500;%迭代次数
    show_prog = 0;
    show_res = 0;
    final_list = fun_mtspofs_ga(xy,dmat,salesmen,min_tour,pop_size,num_iter,...
    show_prog,show_res,start_p,startP,target_p);
    clr = [1 0 0; 0 0 1;  0 1 0; 0.67 0 1; 1 0.5 0];
    if salesmen > 5
        clr = hsv(salesmen);
    end
    path_length=zeros(1,salesmen);%用于统计每条路径的长度
    %
    final_list(1,1:8)=[6 131 74    76   101   122   109   0];
    final_list(2,1:8)=[6 133 117    85    60    64    67  0];
    final_list(3,1:8)=[6 138 35    54    48    24    32  0];
    final_list(4,1:8)=[5 139 124    14     8     5   0  0];
    for i=1:salesmen
        length=final_list(i,1);
        for j=2:length
            startID=final_list(i,j);
            endID=final_list(i,j+1);
             [result_list flag]=ResearchPath(startID,endID,point_inf);
             result_list_length=dis_path(result_list,point_inf );%计算出两节点之间的路径长度
             path_length(1,i)=path_length(1,i)+result_list_length;%路径进行累积
              figure(fig_map)
              hold on
              plot(point_inf(result_list,1),point_inf(result_list,2),'-','Color',clr(i,:),'LineWidth',2);
        end
    end
    %%
     x=800;y=500;
    plot(x, y,'rp','LineWidth',5);
    text(x+30,y,'Start Points','color','r','FontSize',5);
    plot(x, y+30,'gp','LineWidth',5);
    text(x+30,y+30,'Target Points','color','g','FontSize',5);
    x1=[790 820];y1=[560 560];
    i=1;
    plot(x1,y1,'-o','Color',clr(i,:),'LineWidth',1)
     y1=y1+20;
     i=2;
     plot(x1,y1,'-o','Color',clr(i,:),'LineWidth',1) 
     y1=y1+20;
      i=3;
     plot(x1,y1,'-o','Color',clr(i,:),'LineWidth',1) 
     y1=y1+20;
      i=4;
     plot(x1,y1,'-o','Color',clr(i,:),'LineWidth',1) 
     text(x+30,600,'Planned Paths','color','black','FontSize',5);
    %%
     
    fig_map2=figure,imshow(im)
    Show_relationship(point_inf);

    hold on
    plot(point_inf(:,1),point_inf(:,2),'ro');
    plot(xy(:,1)-1,xy(:,2),'gp','LineWidth',2);
    plot(startP(:,1),startP(:,2),'rp','LineWidth',2);
    path_length2=zeros(1,salesmen);%
    for i=1:salesmen
        start_p2=start_p(i);
        startP2=point_inf(start_p2,1:2);
        length=final_list(i,1);
        target_p2=final_list(i,2:length+1);
        xy=point_inf(target_p2,1:2);
        salesmen2 = 1;%旅行商个数
        N = size(xy,1);%城市个数
        dmat=zeros(salesmen2,N,N);
        for j=1:N
            for k=1:N
                start_id= target_p2(j);
                end_id=target_p2(k);
                [result_list flag]=ResearchPath(start_id,end_id,point_inf);
                 result_list_length=dis_path(result_list,point_inf );
                 dmat(1,j,k) = result_list_length;
            end
        end
        final_list2 = fun_mtspofs_ga(xy,dmat,salesmen2,min_tour,pop_size,num_iter,...
    show_prog,show_res,start_p2,startP2,target_p2)
%%
         length=final_list2(1,1);
        
        for j=2:length
            startID=final_list2(1,j);
            endID=final_list2(1,j+1);
            [result_list flag]=ResearchPath(startID,endID,point_inf);
            result_list_length=dis_path(result_list,point_inf );
            path_length2(1,i)=path_length2(1,i)+result_list_length;
            figure(fig_map2)
            plot(point_inf(result_list,1),point_inf(result_list,2),'-','Color',clr(i,:),'LineWidth',2);
        end
    end
     x=800;y=500;
    plot(x, y,'rp','LineWidth',5);
    text(x+30,y,'Start Points','color','r','FontSize',5);
    plot(x, y+30,'gp','LineWidth',5);
    text(x+30,y+30,'Target Points','color','g','FontSize',5);
    x1=[790 820];y1=[560 560];
    i=1;
    plot(x1,y1,'-o','Color',clr(i,:),'LineWidth',1)
     y1=y1+20;
     i=2;
     plot(x1,y1,'-o','Color',clr(i,:),'LineWidth',1) 
     y1=y1+20;
      i=3;
     plot(x1,y1,'-o','Color',clr(i,:),'LineWidth',1) 
     y1=y1+20;
      i=4;
     plot(x1,y1,'-o','Color',clr(i,:),'LineWidth',1) 
     text(x+30,600,'Planned Paths','color','black','FontSize',5);
    
    
    final_list
    path_length
    sum(path_length)
    path_length2
    sum(path_length2)
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

