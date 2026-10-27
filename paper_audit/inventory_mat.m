% INVENTORY_MAT  Dump structure, sizes, time span and initial positions of every .mat (read-only).
files = [dir('../results/*.mat'); dir('../*.mat')];
for f = files'
    p = fullfile(f.folder, f.name);
    fprintf('\n========== %s (%d bytes, modified %s)\n', f.name, f.bytes, f.date);
    S = load(p);
    for v = string(fieldnames(S))'
        x = S.(v);
        fprintf('  var %s : %s %s\n', v, class(x), mat2str(size(x)));
        if isstruct(x) && ~isempty(x)
            for g = string(fieldnames(x))'
                y = x.(g);
                if (isnumeric(y)||islogical(y)) && numel(y) <= 4
                    fprintf('    .%s : %s %s = %s\n', g, class(y), mat2str(size(y)), mat2str(double(y(:)'),6));
                elseif isstring(y) || ischar(y)
                    fprintf('    .%s : %s = "%s"\n', g, class(y), strjoin(string(y),','));
                else
                    fprintf('    .%s : %s %s\n', g, class(y), mat2str(size(y)));
                end
            end
            if isfield(x,'time') && isfield(x,'pursuer_positions')
                t = x.time; dt = diff(t);
                fprintf('    N samples = %d, t = [%.4f, %.4f], dt mean %.4f std %.4f min %.4f max %.4f\n', ...
                    numel(t), t(1), t(end), mean(dt), std(dt), min(dt), max(dt));
                fprintf('    P0 = %s\n', mat2str(x.pursuer_positions(:,1)',4));
                for i = 1:size(x.evader_positions,2)
                    fprintf('    E%d_0 = %s\n', i, mat2str(x.evader_positions(:,i,1)',4));
                end
                if isfield(x,'captured_evaders')
                    for i = 1:size(x.captured_evaders,1)
                        k = find(x.captured_evaders(i,:),1);
                        if isempty(k), fprintf('    E%d never captured\n', i);
                        else, fprintf('    E%d first flagged captured at idx %d, t = %.4f\n', i, k, t(k)); end
                    end
                end
            end
        elseif isnumeric(x) && numel(x) <= 8
            fprintf('    = %s\n', mat2str(x,5));
        end
    end
end
