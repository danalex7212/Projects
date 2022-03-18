function y = im_allign1(x)
  
  
  [row,col] = size(x);
  r = x(1:row/3,:);
  g = x((row/3 + 1):(2*row/3)  ,:);
  b = x(((2*row/3)+1):(row),:);
  
  r2 = im2double(r);
  g2 = im2double(g);
  b2 = im2double(b);
  
  r1 = double(r2(36:306,65:335));
  g1 = double(g2(36:306,65:335));
  b1 = double(b2(36:306,65:335));
  
  error = inf;
  
  for i = -15:15
    for j = -15:15
      shiftg = circshift(g1,[i,j]);
      temp = sum(sum((double(b1)-double(shiftg)).^2));
      if temp < error
        error = temp;
        shift_g_row = i;
        shift_g_col = j;
      end
    end
  end

  error = inf;
  
  for i = -15:15
    for j = -15:15
      shiftr = circshift(r1,[i,j]);
      temp = sum(sum((double(b1)-double(shiftr)).^2));
      if temp < error
        error = temp;
        shift_r_row = i;
        shift_r_col = j;
      end
    end
  end
  
  final_g = circshift(g,[shift_g_row,shift_g_col]);
  final_r = circshift(r,[shift_r_row,shift_r_col]);
  
  fprintf('shift_r_row in SSD = %f\n', shift_r_row);
  fprintf('shift_r_col in SSD = %f\n', shift_r_col);
  fprintf('shift_g_row in SSD = %f\n', shift_g_row);
  fprintf('shift_g_col in SSD = %f\n', shift_g_col);
  
  
  y = cat(3,b,final_g,final_r);
  
end
