close all,clc,clear all
data=load('output.csv');

img=imread('img.png');
[height width  m]=size(img);
imshow(img)
hold on
x=data(:,3);
y=data(:,2);
maxX=max(x);
minX=min(x);
maxY=max(y);
minY=min(y);
kx=width/(maxX-minX);
ky=height/(maxY-minY);
k1=0.45;
k2=0.75;
x=(x-minX)*kx*k1+390;
y=height-(y-minY)*ky*k2-90;
%
% theta=10*pi/180;
% x1=x*cos(theta)-y*sin(theta);
% y1=x*sin(theta)+y*cos(theta);

plot(x,y,'r-o')
% hold on 
% plot(x1,y1,'b-o')
