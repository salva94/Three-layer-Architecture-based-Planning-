function generateLine(line_num,flag)
%生成序列路径点
 %   line_num=[10 9 8 7 6 5 16 17 16 5 4 18 2 1 2 18 13 14  15 14 13 12 11 8 9 10]; %
    file_name='final_data21.csv';
    data=csvread(file_name);

    final_data=[];
    for i=1:size(line_num,2) -1
        
        startP=line_num(i);
        endP=line_num(i+1);
        position=find(((data(:,1)==startP )  & (data(:,2)==endP ))  | ((data(:,2)==startP )  & (data(:,1)==endP )) );
     
        indexP=data(position(1),:);
        get_d=data(position(1)+1:position(1)+indexP(3),:);
        if startP==indexP(2)
            get_d=get_d(end: -1: 1,:);
        end
        final_data=[final_data; get_d];
    end
   % figure,hold on,axis on, axis equal
    plot(final_data(:,3),final_data(:,2),'r-o');
    %去除重合点
    x=final_data(:,3);
    y=final_data(:,2);
    x1=[0; x];    y1=[0; y];    x2=[x; 0];    y2=[y ;0];    dx=x2-x1;    dy=y2-y1;
    dx=dx(2:end-1);    dy=dy(2:end-1);
    dis=sqrt(dy.*dy+dx.*dx);
    maxV=max(dis);
    minV=min(dis);
    averV=sum(dis)/size(dis,1);
    index=find(dis>0);
    final_data=final_data(index,:);
    %去除重合点完毕
    axis on,axis equal 
  plot(final_data(:,3),final_data(:,2),'b-o');
     if flag==1
          file_name='output1.csv';
     elseif flag ==2
             file_name='output2.csv';
     elseif flag ==3
             file_name='output3.csv';
      elseif flag ==4
             file_name='output4.csv';
     elseif flag ==5
             file_name='output5.csv';
     elseif flag ==6
             file_name='output6.csv';
     end
      dlmwrite(file_name,final_data,'precision',18);
end
