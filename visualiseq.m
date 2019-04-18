S = createstatespace;
load('Q.mat')
count = 1;
figure('Units','normalized','Position',[0.1 0.1 0.8 0.8]);
Mx = max(max(max(Q)));
Mn = min(min(min(Q)));
for iState = 1:size(Q,3)
    if count == 10
        figure('Units','normalized','Position',[0.1 0.1 0.8 0.8]);
        count = 1;
    end
    subplot(3,3,count)
    heatmap(Q(:,:,iState),'Colormap',hot,'ColorLimits',[Mn Mx]);
    title(num2str(S{iState}))
    count = count + 1;
end