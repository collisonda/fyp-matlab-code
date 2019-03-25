function createevent(string)
tNow = datetime('now','Format','HH:mm:ss');
if nargin == 0
    disp(['[' char(tNow) '] ***']);
else
    disp(['[' char(tNow) '] ' string]);
end

end

