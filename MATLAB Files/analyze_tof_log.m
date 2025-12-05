function analyze_tof_log(logFiles, config)
% analyze_tof_log  Offline ToF (Time-of-Flight) log analysis on top of
%                  the Automated_CSV_Data_Analysis pipeline.
%
% Usage:
%   analyze_tof_log;                        % GUI: choose CSV(s)
%   analyze_tof_log("tof_log.csv");         % single file
%   analyze_tof_log({"tof1.csv","tof2.csv"}); % multiple files
%
%   cfg = struct;
%   cfg.runPipelineFirst   = true;
%   cfg.pipelineCmd        = 'Automated_CSV_Analysis "%s"';
%   cfg.usePlottableFields = true;
%   cfg.movingWindowSize   = 50;
%   cfg.zScoreThreshold    = 3.0;
%   cfg.jumpThresholdMeters = [];          % auto-choose if empty
%   cfg.refDistanceMeters  = [];           % optional scalar (single known distance)
%   analyze_tof_log("tof_log.csv", cfg);
%
% This function:
%   1) Optionally calls your CSV pipeline on each log file.
%   2) Loads idx, time_s, raw, distance_m, distance_mm (from Plottable_Fields or CSV).
%   3) Performs:
%         - Time-series plots (distance/raw vs time & index)
%         - Descriptive stats
%         - Histograms + normal curve
%         - Boxplots
%         - Moving mean / moving std
%         - Outlier detection (z-score + IQR)
%         - Sudden jump detection (|Δdistance| > threshold)
%         - Optional calibration vs known distance
%         - FFT-based spectral analysis (optional, simple form)
%   4) Writes:
%         - Figures (PNG) into a Figures/ subdirectory
%         - CSV summaries (run_summary.csv, outliers, jumps)
%         - A Markdown report stub linking to the generated figures
%
% All outputs are placed under:
%   <log_dir>/<base>_Plottable_Fields_Full_Analysis_Results/
%       + Figures/
%       + <base>_tof_report.md
%
% so it "bolts onto" your existing pipeline output structure.

    % -------------------------------------------------------------
    % 0. Handle inputs and defaults
    % -------------------------------------------------------------
    if nargin < 1 || isempty(logFiles)
        [f, p] = uigetfile('*.csv', 'Select one or more ToF log CSV files', ...
                           'MultiSelect', 'on');
        if isequal(f,0)
            fprintf('No files selected. Exiting.\n');
            return;
        end
        if ischar(f)
            logFiles = {fullfile(p,f)};
        else
            logFiles = fullfile(p, f);
        end
    end

    if ischar(logFiles) || isstring(logFiles)
        logFiles = {char(logFiles)};
    end

    if nargin < 2
        config = struct;
    end

    cfg = apply_default_config(config);

    % For a multi-log run, we'll accumulate a run summary table:
    runSummary = [];

    % -------------------------------------------------------------
    % 1. Loop over log files
    % -------------------------------------------------------------
    for k = 1:numel(logFiles)
        logPath = logFiles{k};
        if ~isfile(logPath)
            warning('File not found: %s (skipping)', logPath);
            continue;
        end

        [logDir, logBase, ~] = fileparts(logPath);

        fprintf('\n==============================\n');
        fprintf('Analyzing log: %s\n', logPath);
        fprintf('==============================\n');

        % 1a. Optionally invoke the external pipeline
        if cfg.runPipelineFirst
            run_pipeline_on_log(logPath, cfg);
        end

        % 1b. Determine pipeline output directories
        plottableDir   = fullfile(logDir, [logBase '_Plottable_Fields']);
        fullResultsDir = fullfile(logDir, [logBase '_Plottable_Fields_Full_Analysis_Results']);
        figuresDir     = fullfile(fullResultsDir, 'Figures');
        reportsDir     = fullResultsDir;  % keep report at this level

        if ~exist(fullResultsDir, 'dir')
            % If pipeline didn't create it (e.g., running directly on CSV),
            % create it now.
            mkdir(fullResultsDir);
        end
        if ~exist(figuresDir, 'dir')
            mkdir(figuresDir);
        end

        % 1c. Load data
        data = load_tof_numeric_data(logPath, plottableDir, cfg);

        % Data fields: idx, time_s, raw, distance_m, distance_mm
        t   = data.time_s(:);
        idx = data.idx(:);
        raw = data.raw(:);
        d_m = data.distance_m(:);
        d_mm = data.distance_mm(:);

        N = numel(d_m);

        % 1d. Basic stats
        stats = compute_basic_stats(d_m, raw, d_mm, t);

      % 1e. Time-series plots
        figs_ts = make_time_series_plots(t, idx, d_m, d_mm, raw, logBase, figuresDir, cfg);

        % 1f. Noise + distribution (histograms & boxplots)
        figs_dist = make_distribution_plots(d_m, raw, logBase, figuresDir, cfg);

        % 1g. Moving window stats
        [movAvg, movStd, figs_mov] = moving_stats_plots(t, d_m, cfg.movingWindowSize, logBase, figuresDir, cfg);

        % 1h. Outlier detection …
        [outliersTable, outlierIdxSet] = detect_outliers(d_m, t, idx, raw, cfg);
        figs_out_ts = overlay_outliers_on_timeseries(t, d_m, outlierIdxSet, logBase, figuresDir, cfg);

        % 1i. Sudden jump detection
        [jumpsTable, figs_jumps] = detect_jumps(d_m, t, idx, cfg, logBase, figuresDir, cfg);

        % 1j. Calibration
        [calibResult, figs_calib] = single_distance_calibration(d_m, t, cfg.refDistanceMeters, ...
                                                        logBase, figuresDir, cfg);

        % 1k. Spectral analysis
        figs_fft = spectral_analysis(d_m, t, logBase, figuresDir, cfg);

        % 1j. Optional calibration vs single known distance
        calibResult = [];
        figs_calib  = {};
        if ~isempty(cfg.refDistanceMeters)
            [calibResult, figs_calib] = single_distance_calibration(d_m, t, cfg.refDistanceMeters, ...
                                                                    logBase, figuresDir);
        end

        % 1l. Simple spectral analysis (FFT)
        figs_fft = spectral_analysis(d_m, t, logBase, figuresDir);

        % 1m. Assemble run summary row
        runSummary = [runSummary; ...
            build_run_summary_row(logBase, stats, outliersTable, jumpsTable)];

        % 1n. Save outlier & jump tables
        if ~isempty(outliersTable)
            outlierCSV = fullfile(fullResultsDir, sprintf('%s_outliers.csv', logBase));
            writetable(outliersTable, outlierCSV);
        end
        if ~isempty(jumpsTable)
            jumpsCSV = fullfile(fullResultsDir, sprintf('%s_jumps.csv', logBase));
            writetable(jumpsTable, jumpsCSV);
        end

        % 1o. Write Markdown report stub
        reportPath = fullfile(reportsDir, sprintf('%s_tof_report.md', logBase));
        write_markdown_report(reportPath, logBase, stats, cfg, ...
                              figs_ts, figs_dist, figs_mov, figs_out_ts, ...
                              figs_jumps, figs_calib, figs_fft, ...
                              outliersTable, jumpsTable, calibResult);

        fprintf('Analysis complete for %s\n', logBase);
        fprintf('  - Report:  %s\n', reportPath);
        fprintf('  - Figures: %s\n', figuresDir);
    end

    % -------------------------------------------------------------
    % 2. Global run summary CSV for all logs
    % -------------------------------------------------------------
    if ~isempty(runSummary)
        % Place the global summary next to the FIRST log's directory
        [firstDir, ~, ~] = fileparts(logFiles{1});
        summaryPath = fullfile(firstDir, 'ToF_RunSummary.csv');
        writetable(runSummary, summaryPath);
        fprintf('\nGlobal run summary written to:\n  %s\n', summaryPath);
    end
end

% =====================================================================
% Config handling
% =====================================================================
function cfg = apply_default_config(cfg)
    if nargin < 1 || isempty(cfg)
        cfg = struct;
    end

    if ~isfield(cfg, 'runPipelineFirst')
        cfg.runPipelineFirst = false;
    end
    if ~isfield(cfg, 'pipelineCmd')
        % Command template for your C pipeline/exe
        % Adjust to your actual binary name.
        cfg.pipelineCmd = 'Automated_CSV_Analysis "%s"';
    end
    if ~isfield(cfg, 'usePlottableFields')
        cfg.usePlottableFields = true;  % prefer pipeline output
    end
    if ~isfield(cfg, 'movingWindowSize')
        cfg.movingWindowSize = 50;
    end
    if ~isfield(cfg, 'zScoreThreshold')
        cfg.zScoreThreshold = 3.0;
    end
    if ~isfield(cfg, 'jumpThresholdMeters')
        cfg.jumpThresholdMeters = [];   % auto-choose later
    end
    if ~isfield(cfg, 'refDistanceMeters')
        cfg.refDistanceMeters = [];     % optional single known distance (scalar)
    end

    if ~isfield(cfg, 'showFigures')
        % true  => figures are visible and left open
        % false => figures are hidden and closed after saving (batch mode)
        cfg.showFigures = true;
    end

end

% =====================================================================
% Pipeline invoker
% =====================================================================
function run_pipeline_on_log(logPath, cfg)
    cmd = sprintf(cfg.pipelineCmd, logPath);
    fprintf('Running pipeline: %s\n', cmd);
    status = system(cmd);
    if status ~= 0
        warning('Pipeline command returned non-zero status (%d) for %s', status, logPath);
    end
end

% =====================================================================
% Data loader
% =====================================================================
function data = load_tof_numeric_data(logPath, plottableDir, cfg)
    [logDir, logBase, ~] = fileparts(logPath);

    fieldNames = {'idx', 'time_s', 'raw', 'distance_m', 'distance_mm'};

    if cfg.usePlottableFields && exist(plottableDir, 'dir')
        fprintf('Loading fields from Plottable_Fields: %s\n', plottableDir);
        for i = 1:numel(fieldNames)
            fn = fieldNames{i};
            pattern = fullfile(plottableDir, sprintf('*-%s.txt', fn));
            d = dir(pattern);
            if isempty(d)
                error('Could not find plottable field file for "%s" using pattern %s', ...
                      fn, pattern);
            end
            filePath = fullfile(plottableDir, d(1).name);
            vals = readmatrix(filePath, 'FileType', 'text');
            data.(fn) = vals(:);
        end
    else
        fprintf('Loading directly from CSV: %s\n', logPath);
        T = readtable(logPath, 'FileType', 'text');
        for i = 1:numel(fieldNames)
            fn = fieldNames{i};
            if ~ismember(fn, T.Properties.VariableNames)
                error('CSV file %s does not contain required column "%s".', logPath, fn);
            end
            data.(fn) = T.(fn)(:);
        end
    end

    % simple sanity: ensure all same length
    L = numel(data.idx);
    for i = 1:numel(fieldNames)
        if numel(data.(fieldNames{i})) ~= L
            error('Field %s length mismatch in %s.', fieldNames{i}, logPath);
        end
    end
end

% =====================================================================
% Basic statistics
% =====================================================================
function stats = compute_basic_stats(d_m, raw, d_mm, t)
    N = numel(d_m);

    stats.N = N;

    if numel(t) > 1
        stats.duration_s = max(t) - min(t);
    else
        stats.duration_s = 0;
    end

    % Helper
    function s = summarize(vec)
        vec = vec(:);
        s.N        = numel(vec);
        s.mean     = mean(vec, 'omitnan');
        s.median   = median(vec, 'omitnan');
        s.std      = std(vec, 'omitnan');
        s.min      = min(vec, [], 'omitnan');
        s.max      = max(vec, [], 'omitnan');
        s.p5       = prctile(vec, 5);
        s.p25      = prctile(vec, 25);
        s.p75      = prctile(vec, 75);
        s.p95      = prctile(vec, 95);
    end

    stats.distance_m   = summarize(d_m);
    stats.raw          = summarize(raw);
    stats.distance_mm  = summarize(d_mm);
end
function figs = make_time_series_plots(t, idx, d_m, d_mm, raw, base, outDir, cfg)
    figs = {};

    if cfg.showFigures
        figVis = 'on';
    else
        figVis = 'off';
    end

    % Distance vs time
    f1 = figure('Visible', figVis);
    plot(t, d_m, '-');
    hold on;
    plot(t, d_mm / 1000, '.', 'MarkerSize', 6); % overlay mm as meters
    xlabel('Time (s)');
    ylabel('Distance (m)');
    title(sprintf('Distance vs Time - %s', base), 'Interpreter','none');
    legend({'distance\_m','distance\_mm/1000'}, 'Location','best');
    grid on;
    file1 = fullfile(outDir, sprintf('%s_distance_vs_time.png', base));
    exportgraphics(f1, file1);
    figs{end+1} = file1;
    if ~cfg.showFigures
        close(f1);
    end

    % Distance vs index
    f2 = figure('Visible', figVis);
    plot(idx, d_m, '-');
    xlabel('Sample index');
    ylabel('Distance (m)');
    title(sprintf('Distance vs Index - %s', base), 'Interpreter','none');
    grid on;
    file2 = fullfile(outDir, sprintf('%s_distance_vs_index.png', base));
    exportgraphics(f2, file2);
    figs{end+1} = file2;
    if ~cfg.showFigures
        close(f2);
    end

    % Raw vs time
    f3 = figure('Visible', figVis);
    plot(t, raw, '-');
    xlabel('Time (s)');
    ylabel('Raw code (units)');
    title(sprintf('Raw Code vs Time - %s', base), 'Interpreter','none');
    grid on;
    file3 = fullfile(outDir, sprintf('%s_raw_vs_time.png', base));
    exportgraphics(f3, file3);
    figs{end+1} = file3;
    if ~cfg.showFigures
        close(f3);
    end
end


function figs = make_distribution_plots(d_m, raw, base, outDir, cfg)
    figs = {};

    if cfg.showFigures
        figVis = 'on';
    else
        figVis = 'off';
    end

    % Distance histogram + normal curve
    f1 = figure('Visible', figVis);
    h = histogram(d_m, 'Normalization','pdf');
    hold on;
    mu = mean(d_m, 'omitnan');
    s  = std(d_m, 'omitnan');
    if s > 0
        xgrid = linspace(min(d_m), max(d_m), 200);
        ygrid = (1/(s*sqrt(2*pi))) * exp(-0.5*((xgrid - mu)/s).^2);
        plot(xgrid, ygrid, 'LineWidth',1.5);
        legend({'Histogram','Normal(\mu,\sigma)'}, 'Location','best');
    end
    xlabel('Distance (m)');
    ylabel('PDF');
    title(sprintf('Distance Distribution - %s', base), 'Interpreter','none');
    grid on;
    file1 = fullfile(outDir, sprintf('%s_distance_histogram.png', base));
    exportgraphics(f1, file1);
    figs{end+1} = file1;
    if ~cfg.showFigures
        close(f1);
    end

    % Raw histogram
    f2 = figure('Visible', figVis);
    histogram(raw);
    xlabel('Raw code (units)');
    ylabel('Count');
    title(sprintf('Raw Code Histogram - %s', base), 'Interpreter','none');
    grid on;
    file2 = fullfile(outDir, sprintf('%s_raw_histogram.png', base));
    exportgraphics(f2, file2);
    figs{end+1} = file2;
    if ~cfg.showFigures
        close(f2);
    end

    % Boxplots for distance and raw
    f3 = figure('Visible', figVis);
    boxplot([d_m(:), raw(:)], 'Labels',{'distance\_m','raw'});
    ylabel('Value');
    title(sprintf('Boxplots - %s', base), 'Interpreter','none');
    grid on;
    file3 = fullfile(outDir, sprintf('%s_boxplots.png', base));
    exportgraphics(f3, file3);
    figs{end+1} = file3;
    if ~cfg.showFigures
        close(f3);
    end
end




function [movAvg, movStd, figs] = moving_stats_plots(t, d_m, winSize, base, outDir, cfg)
    figs = {};

    if numel(d_m) < winSize
        winSize = max(1, floor(numel(d_m)/4));
    end

    movAvg = movmean(d_m, winSize, 'omitnan');
    movStd = movstd(d_m, winSize, 'omitnan');

    if cfg.showFigures
        figVis = 'on';
    else
        figVis = 'off';
    end

    f1 = figure('Visible', figVis);
    subplot(2,1,1);
    plot(t, movAvg, '-');
    xlabel('Time (s)');
    ylabel(sprintf('Moving mean (win=%d)', winSize));
    grid on;
    title('Moving Average of Distance');

    subplot(2,1,2);
    plot(t, movStd, '-');
    xlabel('Time (s)');
    ylabel(sprintf('Moving std (win=%d)', winSize));
    grid on;
    title('Moving Std of Distance');

    sgtitle(sprintf('Moving Statistics - %s', base), 'Interpreter','none');

    file1 = fullfile(outDir, sprintf('%s_moving_stats.png', base));
    exportgraphics(f1, file1);
    figs{end+1} = file1;
    if ~cfg.showFigures
        close(f1);
    end
end




% =====================================================================
% Outlier detection
% =====================================================================
function [tbl, outlierSet] = detect_outliers(d_m, t, idx, raw, cfg)
    d = d_m(:);
    N = numel(d);

    mu = mean(d, 'omitnan');
    s  = std(d, 'omitnan');

    z = (d - mu) / s;
    zMask = abs(z) > cfg.zScoreThreshold;

    % IQR-based
    q1 = prctile(d, 25);
    q3 = prctile(d, 75);
    iqrVal = q3 - q1;
    lower = q1 - 1.5 * iqrVal;
    upper = q3 + 1.5 * iqrVal;
    iqrMask = (d < lower) | (d > upper);

    outMask = zMask | iqrMask;

    outlierSet = find(outMask);
    fprintf('Outliers detected: %d / %d samples\n', numel(outlierSet), N);

    if isempty(outlierSet)
        tbl = table;
        return;
    end

    tbl = table;
    tbl.index       = idx(outlierSet);
    tbl.time_s      = t(outlierSet);
    tbl.distance_m  = d_m(outlierSet);
    tbl.raw         = raw(outlierSet);
    tbl.zScore      = z(outlierSet);
    tbl.isZScoreOutlier = zMask(outlierSet);
    tbl.isIQROutlier    = iqrMask(outlierSet);
end

function figs = overlay_outliers_on_timeseries(t, d_m, outlierIdxSet, base, outDir, cfg)
    figs = {};

    if cfg.showFigures
        figVis = 'on';
    else
        figVis = 'off';
    end

    f1 = figure('Visible', figVis);
    plot(t, d_m, '-');
    hold on;
    if ~isempty(outlierIdxSet)
        plot(t(outlierIdxSet), d_m(outlierIdxSet), 'o', 'MarkerSize', 6);
        legend({'distance\_m', 'Outliers'}, 'Location','best');
    else
        legend({'distance\_m'}, 'Location','best');
    end
    xlabel('Time (s)');
    ylabel('Distance (m)');
    title(sprintf('Distance vs Time with Outliers - %s', base), 'Interpreter','none');
    grid on;

    file1 = fullfile(outDir, sprintf('%s_distance_vs_time_outliers.png', base));
    exportgraphics(f1, file1);
    figs{end+1} = file1;
    if ~cfg.showFigures
        close(f1);
    end
end


function [tbl, figs] = detect_jumps(d_m, t, idx, cfg, base, outDir, cfgForFig)
    % (You can reuse cfg and ignore cfgForFig, or just pass cfg twice;
    %  for clarity you can collapse to one cfg argument.)
    cfg = cfgForFig;
    figs = {};
    ...
    if cfg.showFigures
        figVis = 'on';
    else
        figVis = 'off';
    end

    f1 = figure('Visible', figVis);
    plot(t(2:end), delta, '-');
    hold on;
    if ~isempty(jumpIdx)
        plot(t(jumpIdx+1), delta(jumpIdx), 'o', 'MarkerSize', 6);
        legend({'\Delta distance','Jumps'}, 'Location','best');
    else
        legend({'\Delta distance'}, 'Location','best');
    end
    xlabel('Time (s)');
    ylabel('\Delta distance (m)');
    title(sprintf('Distance Jumps - %s', base), 'Interpreter','none');
    grid on;
    file1 = fullfile(outDir, sprintf('%s_distance_jumps.png', base));
    exportgraphics(f1, file1);
    figs{end+1} = file1;
    if ~cfg.showFigures
        close(f1);
    end
end



function [calib, figs] = single_distance_calibration(d_m, t, Dtrue, base, outDir, cfg)
    figs = {};
    ...
    if cfg.showFigures
        figVis = 'on';
    else
        figVis = 'off';
    end

    f1 = figure('Visible', figVis);
    histogram(err);
    ...
    exportgraphics(f1, file1);
    figs{end+1} = file1;
    if ~cfg.showFigures
        close(f1);
    end
end



function figs = spectral_analysis(d_m, t, base, outDir, cfg)
    figs = {};
    ...
    if cfg.showFigures
        figVis = 'on';
    else
        figVis = 'off';
    end

    f1 = figure('Visible', figVis);
    plot(f_half, X_half, '-');
    ...
    exportgraphics(f1, file1);
    figs{end+1} = file1;
    if ~cfg.showFigures
        close(f1);
    end
end


% =====================================================================
% Run summary row
% =====================================================================
function row = build_run_summary_row(base, stats, outTbl, jumpsTbl)
    numOutliers = size(outTbl,1);
    numJumps    = size(jumpsTbl,1);

    row = table;
    row.LogFile     = string(base);
    row.Samples     = stats.N;
    row.Duration_s  = stats.duration_s;
    row.Mean_m      = stats.distance_m.mean;
    row.Std_m       = stats.distance_m.std;
    row.Min_m       = stats.distance_m.min;
    row.Max_m       = stats.distance_m.max;
    row.Outliers    = numOutliers;
    row.Jumps       = numJumps;
end

% =====================================================================
% Markdown report writer
% =====================================================================
function write_markdown_report(reportPath, base, stats, cfg, ...
                               figs_ts, figs_dist, figs_mov, figs_out_ts, ...
                               figs_jumps, figs_calib, figs_fft, ...
                               outliersTbl, jumpsTbl, calibResult)

    fid = fopen(reportPath, 'w');
    if fid == -1
        warning('Could not open report file for writing: %s', reportPath);
        return;
    end

    cleanupObj = onCleanup(@() fclose(fid));

    fprintf(fid, '# ToF Log Offline Analysis Report\n\n');
    fprintf(fid, 'Log: **%s**\n\n', base);

    % Summary
    fprintf(fid, '## Run Summary\n\n');
    fprintf(fid, '- Samples: %d\n', stats.N);
    fprintf(fid, '- Duration: %.6f s\n', stats.duration_s);
    fprintf(fid, '- Mean distance: %.4f m\n', stats.distance_m.mean);
    fprintf(fid, '- Std distance: %.4f m\n', stats.distance_m.std);
    fprintf(fid, '- Min / Max distance: [%.4f, %.4f] m\n', ...
        stats.distance_m.min, stats.distance_m.max);
    fprintf(fid, '- Outliers detected: %d\n', size(outliersTbl,1));
    fprintf(fid, '- Sudden jumps detected: %d\n\n', size(jumpsTbl,1));

    if ~isempty(cfg.refDistanceMeters)
        fprintf(fid, '- Reference distance (for calibration): %.4f m\n', cfg.refDistanceMeters);
        fprintf(fid, '- Estimated bias: %.4f m\n', calibResult.bias_m);
        fprintf(fid, '- Noise (std of distance): %.4f m\n\n', calibResult.stdMeasured_m);
    end

    % Time-series
    fprintf(fid, '## Time-Series Analysis\n\n');
    write_fig_list(fid, figs_ts);

    % Distribution / noise
    fprintf(fid, '\n## Noise and Distribution\n\n');
    write_fig_list(fid, figs_dist);

    % Moving stats
    fprintf(fid, '\n## Stability over Time (Moving Stats)\n\n');
    write_fig_list(fid, figs_mov);

    % Outliers
    fprintf(fid, '\n## Outlier and Glitch Detection\n\n');
    write_fig_list(fid, figs_out_ts);
    write_fig_list(fid, figs_jumps);

    if ~isempty(outliersTbl)
        fprintf(fid, '\n### Outlier Table (first 20 rows)\n\n');
        outHead = outliersTbl(1:min(20,height(outliersTbl)), :);
        write_table_as_markdown(fid, outHead);
    end

    if ~isempty(jumpsTbl)
        fprintf(fid, '\n### Sudden Jumps Table (first 20 rows)\n\n');
        jumpsHead = jumpsTbl(1:min(20,height(jumpsTbl)), :);
        write_table_as_markdown(fid, jumpsHead);
    end

    % Calibration
    if ~isempty(figs_calib)
        fprintf(fid, '\n## Calibration (Single Distance)\n\n');
        write_fig_list(fid, figs_calib);
    end

    % Spectral
    if ~isempty(figs_fft)
        fprintf(fid, '\n## Spectral Analysis\n\n');
        write_fig_list(fid, figs_fft);
    end

    fprintf(fid, '\n---\n_Report generated by analyze_tof_log.m_\n');
end

function write_fig_list(fid, figPaths)
    for i = 1:numel(figPaths)
        [~, name, ext] = fileparts(figPaths{i});
        relPath = sprintf('Figures/%s%s', name, ext); % assumes report next to Figures/
        fprintf(fid, '![%s](%s)\n\n', name, relPath);
    end
end

function write_table_as_markdown(fid, T)
    varNames = T.Properties.VariableNames;
    nVars    = numel(varNames);

    % header
    for j = 1:nVars
        fprintf(fid, '| %s ', varNames{j});
    end
    fprintf(fid, '|\n');

    % separator
    for j = 1:nVars
        fprintf(fid, '|---');
    end
    fprintf(fid, '|\n');

    % rows
    for i = 1:height(T)
        for j = 1:nVars
            val = T{i,j};
            if ismissing(val)
                s = '';
            elseif ischar(val) || isstring(val)
                s = char(val);
            elseif isnumeric(val)
                s = num2str(val);
            else
                s = '<obj>';
            end
            fprintf(fid, '| %s ', s);
        end
        fprintf(fid, '|\n');
    end
    fprintf(fid, '\n');
end
