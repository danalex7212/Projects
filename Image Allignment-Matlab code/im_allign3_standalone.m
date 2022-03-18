clear all;
clc;
close all;

tic

im = imread('image3.jpg');
[row,col] = size(im);
r = im(1:row/3,:);
g = im((row/3 + 1):(2*row/3),:);
b = im(((2*row/3)+1):(row),:);
%RO = 0.0
rsize = size(r);
r = im2double(r);

a = 0;
numr =0;

for i = 2:(rsize(1)-1)
  for j = 2:(rsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + r(i+k,j+l);
      end
    end
    r(i,j) = ((1/9)*a);
  end
end

%for i = 2:(rsize(1)-1)
%  for j = 2:(rsize(2)-1)
%    a = 0;
%    for k = -1:1
%      for l = -1:1
%        a= a + r(i+k,j+l);
%      end
%    end
%    r(i,j) = ((1/9)*a);
%  end
%end

gx = [-1,0,1;-2,0,2;-1,0,1];
gy = [-1,-2,-1;0,0,0;1,2,1];

ix = zeros(rsize(1),rsize(2));
iy = zeros(rsize(1),rsize(2));

a = 0;

for i = 2:(rsize(1)-1)
  for j = 2:(rsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + r(i+k,j+l)*gx(2-k,2-l);
      end
    end
    ix(i,j) = a;
  end
end

for i = 2:(rsize(1)-1)
  for j = 2:(rsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + r(i+k,j+l)*gy(2-k,2-l);
      end
    end
    iy(i,j) = a;
  end
end

ix2 = ix.*ix;
iy2 = iy.*iy;
ixy = ix.*iy;

gf = [1,4,7,4,1;4,16,26,16,4;7,26,41,26,7;4,16,26,16,4;1,4,7,4,1]
ro = zeros(rsize(1),rsize(2));
kh = 0.04;
rcorn = [] %zeros(rsize(1),rsize(2));
rcornx = []
rcorny = []
sumr = 0;
%136:206,165:235
for i =  20:(rsize(1)-19)%156:186 %3:rsize(1)-4 %106:236
  for j = 20:(rsize(2)-19)%185:215 %3:rsize(2)-4 %135:265
    m = 1;
    for k = -2:2
      for l = -2:2
        six2(m) = ix2(i+k,j+l)*gf(3-k,3-l);
        siy2(m) = iy2(i+k,j+l)*gf(3-k,3-l);
        sixy(m) = ixy(i+k,j+l)*gf(3-k,3-l);
        m = m+1;
       end
     end
     h = (1/273)*[sum(six2),sum(sixy);sum(sixy),sum(siy2)];
     ro(i,j) = det(h) - (kh*(trace(h))^2);
     sumr = sumr + ro(i,j);
     if abs(ro(i,j)) > 0
       numr = numr + 1;
     end
     
   end
 end
 
 RO = abs(sumr/numr)*2.5;
 
 for i =  20:(rsize(1)-19)
 for j = 20:(rsize(2)-19)
 
 if ro(i,j) > RO
       rcorn (end+1) = ro(i,j) ;
       rcornx(end+1) = i ;
       rcorny(end+1) = j ;
     end
 end
 end
 
 disp('red done')
% for i = 1 : size(ro,1)
%   for j =1 :size(ro,2)
%     if ro(i,j) > 0.8
%       rcorn (end+1) = ro(i,j) ;
%       rcornx(end+1) = i ;
%       rcorny(end+1) = j ;
%     end
%   end
% end
 
%[rcornx, rcorny] = find(ro > 0.2);

%points = [];
%
%for index = 1:size(rcornx,1)
%   
%    rr = rcornx(index);
%    rc = rcorny(index);
%    points = cat(2, points,ro(rr,rc));
%end

%y = row;
%x = col;
 

gsize = size(g);
g = im2double(g);

for i = 2:(gsize(1)-1)
  for j = 2:(gsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + g(i+k,j+l);
      end
    end
    g(i,j) = ((1/9)*a);
  end
end

gx = [-1,0,1;-2,0,2;-1,0,1];
gy = [-1,-2,-1;0,0,0;1,2,1];

ix = zeros(gsize(1),gsize(2));
iy = zeros(gsize(1),gsize(2));

a = 0;

for i = 2:(gsize(1)-1)
  for j = 2:(gsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + g(i+k,j+l)*gx(2-k,2-l);
      end
    end
    ix(i,j) = a;
  end
end

for i = 2:(gsize(1)-1)
  for j = 2:(gsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + g(i+k,j+l)*gy(2-k,2-l);
      end
    end
    iy(i,j) = a;
  end
end

ix2 = ix.*ix;
iy2 = iy.*iy;
ixy = ix.*iy;

gf = [1,4,7,4,1;4,16,26,16,4;7,26,41,26,7;4,16,26,16,4;1,4,7,4,1]
ro = zeros(gsize(1),gsize(2));
kh = 0.04;
gcorn = [];
gcornx = [];
gcorny = [];
sumg = 0;
numg = 0;
%136:206,165:235
for i =  20:(gsize(1)-19)  %3:gsize(1)-4 %106:236%156:186
  for j = 20:(gsize(2)-19)  %3:gsize(2)-4  %135:265 %185:215
    m = 1;
    for k = -2:2
      for l = -2:2
        six2(m) = ix2(i+k,j+l)*gf(3-k,3-l);
        siy2(m) = iy2(i+k,j+l)*gf(3-k,3-l);
        sixy(m) = ixy(i+k,j+l)*gf(3-k,3-l);
        m = m+1;
       end
     end
     h = (1/273)*[sum(six2),sum(sixy);sum(sixy),sum(siy2)];
     ro(i,j) = det(h) - (kh*(trace(h))^2);
     sumg = sumg + ro(i,j);
     if abs(ro(i,j))>0
       numg = numg +1;
     end
     
   end
 end
 
RO = abs(sumg/numg)*2.5;
 
 for i = 20:(gsize(1)-19 )
 for j = 20:(gsize(2)-19 )
 
 if ro(i,j) > RO
       gcorn (end+1) = ro(i,j) ;
       gcornx(end+1) = i ;
       gcorny(end+1) = j ;
     end
 end
end

disp('green done')

bsize = size(b);
b = im2double(b);

for i = 2:(bsize(1)-1)
  for j = 2:(bsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + b(i+k,j+l);
      end
    end
    b(i,j) = ((1/9)*a);
  end
end

gx = [-1,0,1;-2,0,2;-1,0,1];
gy = [-1,-2,-1;0,0,0;1,2,1];

ix = zeros(bsize(1),bsize(2));
iy = zeros(bsize(1),bsize(2));

a = 0;

for i = 2:(bsize(1)-1)
  for j = 2:(bsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + b(i+k,j+l)*gx(2-k,2-l);
      end
    end
    ix(i,j) = a;
  end
end

for i = 2:(bsize(1)-1)
  for j = 2:(bsize(2)-1)
    a = 0;
    for k = -1:1
      for l = -1:1
        a= a + b(i+k,j+l)*gy(2-k,2-l);
      end
    end
    iy(i,j) = a;
  end
end

ix2 = ix.*ix;
iy2 = iy.*iy;
ixy = ix.*iy;

gf = [1,4,7,4,1;4,16,26,16,4;7,26,41,26,7;4,16,26,16,4;1,4,7,4,1]
ro = zeros(bsize(1),bsize(2));
kh = 0.04;
bcorn = [] %zeros(bsize(1),bsize(2));
bcornx = []
bcorny = []
sumb = 0;
numb = 0;
%136:206,165:235
for i =  20:(bsize(1)-19) %156:186  %3:bsize(1)-4 %106:236
  for j = 20:(bsize(2)-19) %185:215  %3:bsize(2)-4  %135:265 
    m = 1;
    for k = -2:2
      for l = -2:2
        six2(m) = ix2(i+k,j+l)*gf(3-k,3-l);
        siy2(m) = iy2(i+k,j+l)*gf(3-k,3-l);
        sixy(m) = ixy(i+k,j+l)*gf(3-k,3-l);
        m = m+1;
       end
     end
     h = (1/273)*[sum(six2),sum(sixy);sum(sixy),sum(siy2)];
     ro(i,j) = det(h) - (kh*(trace(h))^2);
     sumb = sumb + ro(i,j);
     if ro(i,j) > 0
       numb = numb + 1;
      end
   end
 end
 
 RO = abs(sumb/numb)*2.5;
 
 for i = 20:(bsize(1)-19)
 for j = 20:(bsize(2)-19)
 
 if ro(i,j) > RO
       bcorn (end+1) = ro(i,j) ;
       bcornx(end+1) = i ;
       bcorny(end+1) = j ;
     end
 end
end


maxx = [];
maxy = [];
for i = 1 :200
  
  max = 0 ; 
  x = 0;
  for j = 1 : size(rcorn,2)
    if rcorn(1,j) > max
       max = rcorn(1,j);
       x = j;
       
    end 
  end
       rcorn(rcorn == max ) = [];
       maxx(end+1) = rcornx(1,x);
       maxy(end+1) = rcorny(1,x);
end

gmaxx = [];
gmaxy = [];
for i = 1 :200
  
  max = 0 ; 
  x = 0;
  for j = 1 : size(gcorn,2)
    if gcorn(1,j) > max
       max = gcorn(1,j);
       x = j;
       
    end 
  end
       gcorn(gcorn == max ) = [];
       gmaxx(end+1) = gcornx(1,x);
       gmaxy(end+1) = gcorny(1,x);
end


bmaxx = [];
bmaxy = [];
for i = 1 :200
  
  max = 0 ; 
  x = 0;
  for j = 1 : size(bcorn,2)
    if bcorn(1,j) > max
       max = bcorn(1,j);
       x = j;
       
    end 
  end
       bcorn(bcorn == max ) = [];
       bmaxx(end+1) = bcornx(1,x);
       bmaxy(end+1) = bcorny(1,x);
end




rxmid = maxx;
rymid = maxy;

grxmid = gmaxx;
grymid = gmaxy;

bxmid = bmaxx;
bymid = bmaxy;

xthresh = 0;
ythresh = 0;
inline = 0;
max = 0;
xrfinal = 0;
yrfinal = 0;
for i = 1:1000
  
  inline = 0;
  rxc = rxmid;
  ryc = rymid;
  bxc = bxmid;
  byc = bymid;
  rind = randperm(200,2);
  xoff = rxc(rind(2)) - bxc(rind(1));
  yoff = ryc(rind(2)) - byc(rind(1));
 if abs(xoff) <= 12 && abs(yoff) <=12
   
  rxc = rxc .- xoff;
  ryc = ryc .- yoff;
  %rind2 = reshape(randperm(200),100,[]); %randperm(200,2);
  for j = 1:200
    
%    if abs(rxc(rind2(j,1))-bxc(rind2(j,2)))<=xthresh && abs(ryc(rind2(j,1))-byc(rind2(j,2)))<=ythresh
%      inline = inline+1;
%    end
      for k = 1:200%size(rxc)
        if (abs(bxc(j) - rxc(k)) <= xthresh) && (abs(byc(j) - ryc(k)) <=ythresh)
         
          inline = inline + 1 ;
         
%          rxc(k) =[];
%          ryc(k) =[];
         end
      end
  end
    if inline > max
    max = inline
    i
    xrfinal = xoff;
    yrfinal = yoff;
    end
end

%    for j = 1:size(rxc,2)
%     
%       inline = sum(sum((rxc(1,j) == bxc(1,j))));%.*(ryc(1,j) == byc(1,:))));
%        
%       if inline > max
%        max = inline;
%        i
%        xrfinal = xoff;
%        yrfinal = yoff;
%      end
%    
%    end
  
end
xrfinal,yrfinal
%max,xrfinal,yrfinal
xthresh = 0;
ythresh = 0;
inline = 0;
max = 0;
xgfinal = 0;
ygfinal = 0;

for i = 1:1000
  
  inline = 0;
  gxc = grxmid;
  %gxc(1:10)
  gyc = grymid;
  bxc = bxmid;
  byc = bymid;
  rind = randperm(200,2);
  xoff = gxc(rind(2)) - bxc(rind(1));
  yoff = gyc(rind(2)) - byc(rind(1));
 if abs(xoff) <= 12 && abs(yoff) <=12
  gxc = gxc .- xoff;
  gyc = gyc .- yoff;
  %rind2 = reshape(randperm(200),100,[]); %randperm(200,2);
  for j = 1:200
    %rind2 = randperm(200,2);
%    if abs(gxc(rind2(j,1))-bxc(rind2(j,2)))<=xthresh && abs(gyc(rind2(j,1))-byc(rind2(j,2)))<=ythresh
%      inline = inline+1;
%    end
       for k = 1:200%size(gxc)
        if (abs(bxc(j) - gxc(k))<= xthresh) && abs(byc(j) - gyc(k)) <= ythresh
          inline = inline + 1 ;
          %gxc(k) =[];
          %gyc(k) =[];
         end
      end
  end
  if inline > max
    max = inline
    i
    xgfinal = xoff;
    ygfinal = yoff;
  end
 end
%  for j = 1:size(gxc,2)
%     
%       inline = sum(sum((gxc(1,j) == bxc(1,j))));%.*(gyc(1,j) == byc(1,:))));
%        
%       if inline > max
%        max = inline;
%        i
%        xgfinal = xoff;
%        ygfinal = yoff;
%      end
%    
%    end
end

xgfinal,ygfinal

sample_r = circshift(im(1:row/3,:),[-1*xrfinal,-1*yrfinal]);
sample_g = circshift(im((row/3 + 1):(2*row/3),:),[-1*xgfinal,-1*ygfinal]);
imwrite(cat(3,im(((2*row/3)+1):(row),:),sample_g,sample_r),'image3-corner.jpg');

 toc