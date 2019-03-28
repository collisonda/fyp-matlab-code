load('Q.mat')
for iState = 1:size(Q,3)
   figure
   heatmap(Q(:,:,iState),'Colormap',hot,'ColorLimits',[-100 100]);
end