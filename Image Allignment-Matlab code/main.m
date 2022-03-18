tic

clear all;

imgname = 'image2'
x = imread(strcat(imgname,'.jpg'));
y = im_allign1(x);
imwrite(y,strcat(imgname,'-ssd.jpg'));
x = imread(strcat(imgname,'.jpg'));
z = im_allign2(x);
imwrite(z,strcat(imgname,'-ncc.jpg'));
clear x;
im = imread(strcat(imgname,'.jpg'));
w = im_allign3(im);
imwrite(w,strcat(imgname,'-corner.jpg'));

toc