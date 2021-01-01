% 
% k = 5;
% x = 50.*rand(1,k)+25;
% y = 50.*rand(1,k)+25;
% dists = cell2mat(arrayfun(@(K) bsxfun(@(J,K) hypot(x(J)-x(K),y(J)-y(K)), K, 1:k), 1:k, 'Uniform',0).');
% [sorteddists, rowidx] = sort(dists,2);
% rowidx
% x = x(rowidx(1,2))
% y = y(rowidx(1,2))]
% % n = rowidx(1,2)
% % 
% 
% % for i=1:5
% %     
% % end
% 
% % x = 1:.5:56  
% % y = 1:.5:56
% % 
% % x = x-x(1)
% % y = y-y(1)
% % 
% % x = x./5*x(end)
% % y = y./5*y(end)
% % 
% % plot(x,y)


Sran = linspace(1,10,5)
Sran = uint8(Sran)
