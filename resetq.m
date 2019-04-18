function [] = resetq()
%RESETQ Loads a blank Q and visitCount and overwrites existing Q and
%visitCount.
%       Author: Dale Collison

load('untrainedQ.mat','Q');
save('Q.mat','Q');
load('blankVisitCount.mat','visitCount');
save('visitCount.mat','visitCount');
end
