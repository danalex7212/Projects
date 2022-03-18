function z = im_allign2(x)
  
  [row,col] = size(x);
  r = x(1:row/3,:);
  g = x((row/3 + 1):(2*row/3)  ,:);
  b = x(((2*row/3)+1):(row),:);
  
  r2 = im2double(r);
  g2 = im2double(g);
  b2 = im2double(b.*0.2);
  
  r1 = double(r2(36:306,65:335));
  g1 = double(g2(36:306,65:335));
  b1 = double(b2(36:306,65:335));
  
  #image(r2);
  #imwrite(g2,'g1.jpg');
  #imwrite(b2,'b1.jpg');
  
  error = -1;
  
  for i = -15:15
    for j = -15:15
      shiftg = circshift(g1,[i,j]);
      meang = double((sum(sum(shiftg)))/2500);
      meanb = double((sum(sum(b1)))/2500);
      subg = double(shiftg-meang);
      subb = double(b1 - meanb);
      numer = double(sum(sum(subg .* subb)));
      subg2 = double((shiftg-meang).^2);
      subb2 = double((b1 - meanb).^2);
      denom = double((sum(sum(subb2))*sum(sum(subg2)))^0.5);
      temp = double(numer/denom);
      
      if temp > error
        error = double(temp);
        shift_g_row = i;
        shift_g_col = j;
      end
    end
  end

  error = -1 ;

  for i = -15:15
    for j = -15:15
      shiftr = circshift(r1,[i,j]);
      meanr = double((sum(sum(shiftr)))/2500);
      meanb = double((sum(sum(b1)))/2500);
      subr = double(shiftr-meanr);
      subb = double(b1 - meanb);
      numer = double(sum(sum(subr .* subb)));
      subr2 = double((shiftr-meanr).^2);
      subb2 = double((b1 - meanb).^2);
      denom = double((sum(sum(subr2))*sum(sum(subb2)))^0.5);
      temp = double(numer/denom);
      if temp > error
        error = double(temp);
        shift_r_row = i;
        shift_r_col = j;
      end
    end
  end

  final_g = circshift(g,[shift_g_row,shift_g_col]);
  final_r = circshift(r,[shift_r_row,shift_r_col]);
  
  fprintf('shift_r_row in NCC= %f\n', shift_r_row);
  fprintf('shift_r_col in NCC= %f\n', shift_r_col);
  fprintf('shift_g_row in NCC= %f\n', shift_g_row);
  fprintf('shift_g_col in NCC= %f\n', shift_g_col);
  
  
  z = cat(3,b,final_g,final_r);
  
end
