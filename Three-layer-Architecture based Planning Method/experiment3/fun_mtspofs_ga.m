function final_list = fun_mtspofs_ga(xy,dmat,salesmen,min_tour,pop_size,num_iter,show_prog,show_res,start_p,startP,target_p)
%% 遗传算法写成函数的形式，方便调用和级联
clr = [1 0 0; 0 0 1;  0 1 0; 0.67 0 1; 1 0.5 0];
if salesmen > 5
    clr = hsv(salesmen);
end
Vari_num=8;%变异分支
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
        if dims == 3, plot3(xy(1,1),xy(1,2),xy(1,3),'ko');
        else plot(xy(1,1),img_height-xy(1,2),'ko'); end
        subplot(2,2,4);
        plot(dist_history,'b','LineWidth',2);
        title('Best Solution History');
        set(gca,'XLim',[0 num_iter+1],'YLim',[0 1.1*max([1 dist_history])]);
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
    % Return Outputs
   