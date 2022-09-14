function varargout = mtspofs_ga(xy,dmat,salesmen,min_tour,pop_size,num_iter,show_prog,show_res)
%时间：20200618
%作者：sek
%用途：用于DMT多无人车自主协同规划算法
%思路：多无人车，多目标点，无人车必须要经过目标点，给出分配方案，用A星搜索所有两点之间的路网和代价，用遗传
%算法来分配路网
% 分别用最长路径最短和总路径最短这两个优化条件，来实现路径规划

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
file_name='map2.csv';
point_inf=dlmread(file_name);
point_inf(:,7)=10000;
Show_relationship(point_inf);
target_p=[85 85 60 48 101 54 32 24 67 64 35 109 76 5 14 74 117 122 13];%目标位置，第一个位置没用
xy=point_inf(target_p,1:2);
start_p=[131 133 136 139];%无人车位置
startP=point_inf(start_p,1:2);

hold on
plot(point_inf(:,1),point_inf(:,2),'ro');
plot(xy(:,1)-1,xy(:,2),'gp','LineWidth',5);
plot(startP(:,1),startP(:,2),'rp','LineWidth',5);
% Process Inputs and Initialize Defaults

nargs = 8;
Vari_num=8;%变异分支
for k = nargin:nargs-1
    switch k
        case 0

        case 1
            salesmen = size(startP,1);%旅行商个数
         case 2
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

        case 3
            min_tour = 1;%每个人最小的长度
        case 4
            pop_size = Vari_num*800;%人口数量，需要是变异种类的倍数
        case 5
            num_iter = 500;%迭代次数
        case 6
            show_prog = 0;
        case 7
            show_res = 0;
        otherwise
    end
end
%dmat=dmat1;
% Verify Inputs
clr = [1 0 0; 0 0 1;  0 1 0; 0.67 0 1; 1 0.5 0];
if salesmen > 5
    clr = hsv(salesmen);
end
final_list=GA();
for i=1:salesmen
    length=final_list(i,1);
    for j=2:length
        startID=final_list(i,j);
        endID=final_list(i,j+1);
         [result_list flag]=ResearchPath(startID,endID,point_inf);
          figure(fig_map)
          hold on
          plot(point_inf(result_list,1),point_inf(result_list,2),'-','Color',clr(i,:),'LineWidth',5);
    end
end
%%
% for i=1:salesmen
%     length=final_list(i,1);
%     target_p=final_list(i,2:length+1);
%     xy=point_inf(target_p,1:2);
    
function final_list=GA()
    [N,dims] = size(xy);
    % [nr,nc] = size(dmat);
    % if N ~= nr || N ~= nc
    %     error('Invalid XY or DMAT inputs!')
    % end
    n = N - 1; % Separate Start City

    % Sanity Checks
    salesmen = max(1,min(n,round(real(salesmen(1)))));
    min_tour = max(1,min(floor(n/salesmen),round(real(min_tour(1)))));
    pop_size = max(Vari_num,Vari_num*ceil(pop_size(1)/Vari_num));
    num_iter = max(1,round(real(num_iter(1))));
    show_prog = logical(show_prog(1));
    show_res = logical(show_res(1));

    % Initializations for Route Break Point Selection
    num_brks = salesmen-1;
    dof = n - min_tour*salesmen;          % degrees of freedom
    addto = ones(1,dof+1);
    for k = 2:num_brks
        addto = cumsum(addto);
    end
    cum_prob = cumsum(addto)/sum(addto);

    % Initialize the Populations
    %初始化种群
    %随机产生序列号
    pop_rte = zeros(pop_size,n);          % population of routes
    pop_brk = zeros(pop_size,num_brks);   % population of breaks
    for k = 1:pop_size
        pop_rte(k,:) = randperm(n)+1;%随机出种群点的序号
        pop_brk(k,:) = randbreaks();%随机产生断开点，这个点表示分配给某个人的个数
    end

    % Select the Colors for the Plotted Routes
    

    % Run the GA
    global_min = Inf;
    total_dist = zeros(1,pop_size);%每个种子下n个saleman需要的总距离
    max_saleman_dist = zeros(1,pop_size);%每个种子下最大距离的saleman需要的距离
    min_saleman_dist = zeros(1,pop_size);%每个种子下最小距离的saleman需要的距离
    dist_history = zeros(1,num_iter);
    tmp_pop_rte = zeros(8,n);
    tmp_pop_brk = zeros(8,num_brks);
    new_pop_rte = zeros(pop_size,n);
    new_pop_brk = zeros(pop_size,num_brks);
    if show_prog
        pfig = figure('Name','MTSPOFS_GA | Current Best Solution','Numbertitle','off');
    end
    for iter = 1:num_iter
        % Evaluate Members of the Population

        dis_saleman=zeros(salesmen,1);
        for p = 1:pop_size
            d = 0;
            p_rte = pop_rte(p,:);%取每一组种子
            p_brk = pop_brk(p,:);
            rng = [[1 p_brk+1];[p_brk n]]';
            %这个就是建模过程，必须得看懂，才能改的动

            for s = 1:salesmen
                d =  dmat(s,1,p_rte(rng(s,1)));
                for k = rng(s,1):rng(s,2)-1
                    d = d + dmat(s,p_rte(k),p_rte(k+1));
                end
                dis_saleman(s)=d;
            end
            %total_dist(p) = d;
            total_dist(p)=sum(dis_saleman);%求得每个种子下所有人路径的总和，
            max_saleman_dist(p)=max(dis_saleman);%求得每个种子下最长路径那个人的路径长度
            min_saleman_dist(p)=min(dis_saleman);%求得每个种子下最短路径那个人的路径长度
        end
        %用来评价的数据；可以是最大旅行商的最小值，也可以是总路程最短，也可以是最大和最小之差最小
       %Evaluation_Unit=total_dist;
       Evaluation_Unit=max_saleman_dist;
        % Find the Best Route in the Population
        [min_dist,index] = min(Evaluation_Unit);%把评价数组中找最小的值，认为是最优的
        dist_history(iter) = min_dist;
        if min_dist < global_min
            global_min = min_dist;
            opt_rte = pop_rte(index,:);
            opt_brk = pop_brk(index,:);
            rng = [[1 opt_brk+1];[opt_brk n]]';
            if show_prog
                % Plot the Best Route
                figure(pfig);
                for s = 1:salesmen

                    xy(1,:)=startP(s,:);

                    rte = [1 opt_rte(rng(s,1):rng(s,2))];
                    if dims == 3, plot3(xy(rte,1),xy(rte,2),xy(rte,3),'.-','Color',clr(s,:));
                    else plot(xy(rte,1),img_height-xy(rte,2),'.-','Color',clr(s,:)); end
                    title(sprintf('Total Distance = %1.4f, Iteration = %d',min_dist,iter));
                    hold on
                end
                if dims == 3, plot3(xy(1,1),xy(1,2),xy(1,3),'ko');
                else
                    for i=1:size(startP,1)
                        plot(startP(i,1),img_height-startP(i,2),'ko'); 
                    end
                end
                hold off
            end
        end

        % Genetic Algorithm Operators
        rand_grouping = randperm(pop_size);%对种群随机分组
        for p = Vari_num:Vari_num:pop_size

            rtes = pop_rte(rand_grouping(p-Vari_num+1:p),:);%种子库里随机取一组
            brks = pop_brk(rand_grouping(p-Vari_num+1:p),:);
            dists = Evaluation_Unit(rand_grouping(p-Vari_num+1:p));
            [ignore,idx] = min(dists);%选择取出来的组里最好的种子
            best_of_8_rte = rtes(idx,:);
            best_of_8_brk = brks(idx,:);
            rte_ins_pts = sort(ceil(n*rand(1,2)));
            I = rte_ins_pts(1);
            J = rte_ins_pts(2);
            for k = 1:Vari_num % Generate New Solutions，对最优的这组开始变异成多组，k=1时为保留自己
                tmp_pop_rte(k,:) = best_of_8_rte;
                tmp_pop_brk(k,:) = best_of_8_brk;
                switch k
                    case 2 % Flip翻转
                        tmp_pop_rte(k,I:J) = fliplr(tmp_pop_rte(k,I:J));
                    case 3 % Swap交换
                        tmp_pop_rte(k,[I J]) = tmp_pop_rte(k,[J I]);
                    case 4 % Slide
                        tmp_pop_rte(k,I:J) = tmp_pop_rte(k,[I+1:J I]);
                    case 5 % Modify Breaks
                        tmp_pop_brk(k,:) = randbreaks();
                    case 6 % Flip, Modify Breaks
                        tmp_pop_rte(k,I:J) = fliplr(tmp_pop_rte(k,I:J));
                        tmp_pop_brk(k,:) = randbreaks();
                    case 7 % Swap, Modify Breaks
                        tmp_pop_rte(k,[I J]) = tmp_pop_rte(k,[J I]);
                        tmp_pop_brk(k,:) = randbreaks();
                    case 8 % Slide, Modify Breaks
                        tmp_pop_rte(k,I:J) = tmp_pop_rte(k,[I+1:J I]);
                        tmp_pop_brk(k,:) = randbreaks();
                    otherwise % Do Nothing
                end
            end
            new_pop_rte(p-Vari_num+1:p,:) = tmp_pop_rte;
            new_pop_brk(p-Vari_num+1:p,:) = tmp_pop_brk;
        end
        pop_rte = new_pop_rte;
        pop_brk = new_pop_brk;
    end
    final_list=zeros(salesmen,N);
    if show_res==0
        rng = [[1 opt_brk+1];[opt_brk n]]';
        for s = 1:salesmen
            rte = [1 opt_rte(rng(s,1):rng(s,2))];
            list=[start_p(s) target_p(opt_rte(rng(s,1):rng(s,2)))];
            final_list(s,1)=size(list,2);
            final_list(s,2:size(list,2)+1)=list;
        end
    end

    if show_res
    % Plots
        figure('Name','MTSPOFS_GA | Results','Numbertitle','off');
        subplot(2,2,1);
        if dims == 3, plot3(xy(:,1),xy(:,2),xy(:,3),'k.');
        else plot(xy(:,1),img_height-xy(:,2),'k.'); end
        hold on
        plot(startP(:,1),img_height-startP(:,2),'ro');
        hold off
        title('City Locations');

       % subplot(2,2,2);
       % imagesc(dmat(1,[1 opt_rte],[1 opt_rte]));
      %  title('Distance Matrix');
        subplot(2,2,3);
        rng = [[1 opt_brk+1];[opt_brk n]]';

        for s = 1:salesmen
            xy(1,:)=startP(s,:);
            rte = [1 opt_rte(rng(s,1):rng(s,2))];
            list=[start_p(s) target_p(opt_rte(rng(s,1):rng(s,2)))];
            final_list(s,1)=size(list,2);
            final_list(s,2:size(list,2)+1)=list;
            if dims == 3, plot3(xy(rte,1),xy(rte,2),xy(rte,3),'.-','Color',clr(s,:));
            else plot(xy(rte,1),img_height-xy(rte,2),'.-','Color',clr(s,:)); end
            title(sprintf('Total Distance = %1.4f',min_dist));
            d=0;
            for k = 1:size(rte,2)-1
                d = d + dmat(s,rte(k),rte(k+1));
            end
            dis_saleman(s,1:2)=[start_p(s) d]; 
            hold on;
        end
        dis_saleman
        sum(dis_saleman)
        if dims == 3, plot3(xy(1,1),xy(1,2),xy(1,3),'ko');
        else plot(xy(1,1),img_height-xy(1,2),'ko'); end
        subplot(2,2,4);
        plot(dist_history,'b','LineWidth',2);
        title('Best Solution History');
        set(gca,'XLim',[0 num_iter+1],'YLim',[0 1.1*max([1 dist_history])]);
    end

    % Return Outputs
    if nargout
        varargout{1} = opt_rte;
        varargout{2} = opt_brk;
        varargout{3} = min_dist;
    end
    function breaks = randbreaks()
        if min_tour == 1 % No Constraints on Breaks
            tmp_brks = randperm(n-1);
            breaks = sort(tmp_brks(1:num_brks));
        else % Force Breaks to be at Least the Minimum Tour Length
            num_adjust = find(rand < cum_prob,1)-1;
            spaces = ceil(num_brks*rand(1,num_adjust));
            adjust = zeros(1,num_brks);
            for kk = 1:num_brks
                adjust(kk) = sum(spaces == kk);
            end
            breaks = min_tour*(1:num_brks) + cumsum(adjust);
        end
    end
end
    % Generate Random Set of Break Points
    
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
