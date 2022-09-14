function gplan_generateMap_GA()
clear all;clc;close all
%% input width and height of the map 
%20210130����·�����޸�·��
%
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
%im=rgb2gray(im);
figure,imshow(im,[])
hold on
file_name='map2.csv';
point_inf=[];
disp('�Ƿ����е�ͼ��1���ǣ�2����')
command=input('');
if command ==1
    point_inf=dlmread(file_name);
    Show_relationship(point_inf);
end
data=[];

%%
while(1)
   disp('����ʲô����1��ѡ��˳��ؼ��㣻2��ѡ�����ӹ����ؼ��㣻3��������������ӹ�ϵ��4��ȥ����������ӹ�ϵ��5:�鿴��ͼ��6��ȥ���ؼ��㣻0���˳�')
   command=input('');
   if command ==0
       break;
   end
   switch command
       case {1}
            disp('������ѡ��˳��ؼ���,�Զ��������㣬���Ҽ��˳�')
            point_inf=SelectOrderNode(data,point_inf);
       case {2}
            disp('������ѡ�����ӹ����ؼ���,���Ҽ��˳�')
            point_inf=SelectSingleNode(data,point_inf);
       case {3}
            disp('������ѡ�������ؼ���,�Զ����������ϵ�����Ҽ��˳�')
            point_inf=AddTwoNode(point_inf);
       case {4}
            disp('������ѡ�������ؼ���,ȡ������֮���ϵ�����Ҽ��˳�')
            point_inf=DeleteTwoNodeRelationship(point_inf);
       case {5}
            figure
            Show_relationship(point_inf);
       case {6}
            disp('������ѡ����Ҫɾ���Ĺؼ���,���Ҽ��˳�')
            %disp('���ܲ��ȶ�����ʱ���ܶ���ʹ��')
            point_inf=DeleteNode(point_inf);
   end
  
   
end
  figure,
   Show_relationship(point_inf); 
   disp('�Ƿ񱣴��ͼ��1�����棬2��������')
   command=input('');
   if command==1
       point_inf(:,f_index)=10000;
       dlmwrite(file_name,point_inf,'precision',18);
   end
%%  
%   
end

function  point_inf=DeleteNode(point_inf)
global x_index;
global y_index;
global self_id_index;
global son_number_index;
hold on
    buttom=0;
    while(1)
       [x y buttom]=ginput(1);
       if buttom==3
           break;
       end
       [d col1]=min(abs(point_inf(:,x_index)-x(1))+abs(point_inf(:,y_index)-y(1)));
       plot(point_inf(col1,x_index),point_inf(col1,y_index),'r*');
       for i=1:point_inf(col1,son_number_index)
           temp_id=point_inf(col1,son_number_index+i);
           son_id=find(point_inf(:,self_id_index)==temp_id);
           tx=[point_inf(col1,x_index) point_inf(son_id,x_index)];
           ty=[point_inf(col1,y_index) point_inf(son_id,y_index)];
           plot(tx,ty,'r-*');
           for j=1:point_inf(son_id,son_number_index)
               if  point_inf(son_id,son_number_index+j)==point_inf(col1,self_id_index)
                    point_inf(son_id,son_number_index+j)=point_inf(son_id,self_id_index);
               end
           end
       end
       point_inf(col1,:)=[]; 
    end
    hold off

end
function point_inf=DeleteTwoNodeRelationship(point_inf)
global x_index;
global y_index;
global self_id_index;
global son_number_index;
    hold on
    buttom=0;
    while(1)
       [x y buttom]=ginput(2);
       if buttom==3
           break;
       end
       [d col1]=min(abs(point_inf(:,x_index)-x(1))+abs(point_inf(:,y_index)-y(1)));
       [d col2]=min(abs(point_inf(:,x_index)-x(2))+abs(point_inf(:,y_index)-y(2)));
         tx=[point_inf(col1,x_index) point_inf(col2,x_index)];
         ty=[point_inf(col1,y_index) point_inf(col2,y_index)];
         plot(tx,ty,'r-*');
       for i=1:point_inf(col1,son_number_index)
            if point_inf(col2,self_id_index)==point_inf(col1,son_number_index+i)
                point_inf(col1,son_number_index+i)=point_inf(col1,self_id_index);
            end
       end
        for i=1:point_inf(col2,son_number_index)
            if point_inf(col1,self_id_index)==point_inf(col2,son_number_index+i)
                point_inf(col2,son_number_index+i)=point_inf(col2,self_id_index);
            end
        end
    end
    hold off
end

function point_inf=SelectSingleNode(data,point_inf)
global x_index;
global y_index;
global self_id_index;
global son_number_index;
global f_index;
    hold on

    while(1)
        [x y bottom]=ginput(1);
        if bottom==3
            break;
        end
%         d=abs(data(:,2)-x)+abs(data(:,3)-y);
%         [value col]=min(d);
       
        point_inf(end+1,x_index)=x;
        point_inf(end,y_index)=y;
        if size(point_inf,1)<2
            point_inf(end,self_id_index)=1;
        else
            point_inf(end,self_id_index)=max(point_inf(:,self_id_index))+1;
        end
        %point_inf(end,f_index)=1000;
        point_inf(end,son_number_index)=0;
        plot(point_inf(end,x_index),point_inf(end,y_index),'go');
    end
    
    hold off
   
end

function point_inf=SelectOrderNode(data,point_inf)
global x_index;
global y_index;
global self_id_index;
global son_number_index;
global f_index;
    hold on
    first_point_flag = 1;
    while(1)
        [x y bottom]=ginput(1);
        if bottom==3
            break;
        end
%         d=abs(data(:,2)-x)+abs(data(:,3)-y);
%         [value col]=min(d);
       
        point_inf(end+1,x_index)=x;
        point_inf(end,y_index)=y;
        if size(point_inf,1)<2% the first point
            point_inf(end,self_id_index)=1;
        else
            point_inf(end,self_id_index)=max(point_inf(:,self_id_index))+1;
        end
        if first_point_flag==1
            point_inf(end,son_number_index)=0;
            plot(point_inf(end,x_index),point_inf(end,y_index),'go');
        else
            
            point_inf(end,son_number_index)=1;
            point_inf(end,son_number_index+1)= point_inf(end-1,self_id_index);
            %point_inf(end,f_index)=1000;
            point_inf(end-1,son_number_index)=point_inf(end-1,son_number_index)+1;
            point_inf(end-1,son_number_index+point_inf(end-1,son_number_index))= point_inf(end,self_id_index);
            tx=[point_inf(end,x_index) point_inf(end-1,x_index)];
            ty=[point_inf(end,y_index) point_inf(end-1,y_index)];
            plot(tx,ty,'g-o');
        end
        first_point_flag = 0;
    end
    
    hold off
   
end

function point_inf=AddTwoNode(point_inf)
    global x_index;
    global y_index;
    global self_id_index;
    global son_number_index;

    buttom=0;
    hold on
    while(1)
       [x y buttom]=ginput(2);
       if buttom==3
           break;
       end
       [d col1]=min(abs(point_inf(:,x_index)-x(1))+abs(point_inf(:,y_index)-y(1)));
       [d col2]=min(abs(point_inf(:,x_index)-x(2))+abs(point_inf(:,y_index)-y(2)));
        tx=[point_inf(col1,x_index) point_inf(col2,x_index)];
        ty=[point_inf(col1,y_index) point_inf(col2,y_index)];
        plot(tx,ty,'g-o');
        point_inf(col1,son_number_index)=point_inf(col1,son_number_index)+1;
        point_inf(col1,son_number_index+point_inf(col1,son_number_index))= point_inf(col2,self_id_index);
        point_inf(col2,son_number_index)=point_inf(col2,son_number_index)+1;
        point_inf(col2,son_number_index+point_inf(col2,son_number_index))= point_inf(col1,self_id_index);
    end
    hold off
end

   
%    while(1)
%         [x y buttom]=ginput(2);
%        if buttom ==3
%            disp('get over of cancal relationshipship!')
%            break;
%        end
%        disp('continue cancal relationshipship!')
%        dis1=[];
%        dis2=[];
%        for i=1:input_width*input_height
%            x1=point_inf(i,x_index);
%            y1=point_inf(i,y_index);
%            dis=sqrt((x1-x(1))^2+(y1-y(1))^2);
%            dis1=[dis1 dis];
%            dis=sqrt((x1-x(2))^2+(y1-y(2))^2);
%            dis2=[dis2 dis];
%        end
%        [data p1]=min(dis1);
%        [data p2]=min(dis2);
%        for i=1:point_inf(p1,son_number_index)
%             if p2==point_inf(p1,son_number_index+i)
%                 point_inf(p1,son_number_index+i)=point_inf(p1,self_id_index);
%             end
%        end
%         for i=1:point_inf(p2,son_number_index)
%             if p1==point_inf(p2,son_number_index+i)
%                 point_inf(p2,son_number_index+i)=point_inf(p2,self_id_index);
%             end
%         end
%       
%        
%        Show_relationship(point_inf);
%    end
%   
% end
       

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