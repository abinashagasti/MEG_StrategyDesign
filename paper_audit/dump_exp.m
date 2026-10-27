P = '/Users/abinashagasti/IITM/Research/Paper Submissions/Multiplayer Evader/Modified Experiment Images/';
for f = {'Experiment 1/Exp1_3.mat','Experiment 2/Exp2_1.mat'}
    fprintf('\n===== %s\n', f{1});
    S = load([P f{1}]);
    disp(S)
    for v = string(fieldnames(S))'
        x = S.(v);
        if isstruct(x), disp(x); for g = string(fieldnames(x))', y = x.(g); fprintf('.%s %s %s\n', g, class(y), mat2str(size(y))); if numel(y)<=6, disp(y); end, end, end
    end
    h = S.history;
    fprintf('time:\n'); fprintf('%.4f ', h.time); fprintf('\n');
    fprintf('dt:\n'); fprintf('%.4f ', diff(h.time)); fprintf('\n');
    disp(h.pursuer_positions(:,1:3)); disp(squeeze(h.evader_positions(:,:,1)));
    disp(double(h.captured_evaders));
end
