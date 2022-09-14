file_name='map2.csv';
file_name1='map3.csv';
point_inf=dlmread(file_name);
point_inf(:,7)=10000;
dlmwrite(file_name1,point_inf,'precision',8);