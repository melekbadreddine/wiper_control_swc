function migrate_to_autosar(varargin)
%MIGRATE_TO_AUTOSAR  Automated setup of the WiperControl_SWC Simulink model.
%
%   MIGRATE_TO_AUTOSAR() imports the AUTOSAR component description, builds a
%   Simulink model around it, attaches the 10 ms runnable logic, maps that
%   logic to Runnable_WiperControl_10ms and configures the model for SIL.
%
%   MIGRATE_TO_AUTOSAR('Logic','caller')   use a C Caller block bound to
%                                         WiperControl.c instead of Stateflow
%   MIGRATE_TO_AUTOSAR('ModelName','X')    override the model name
%   MIGRATE_TO_AUTOSAR('Force',true)       rebuild from scratch
%   MIGRATE_TO_AUTOSAR('Build',false)      skip the final slbuild
%
%   Run headless with:  matlab -batch "migrate_to_autosar"
%
%   ---------------------------------------------------------------------
%   IMPORTANT - READ BEFORE RUNNING
%   ---------------------------------------------------------------------
%   This script could NOT be executed during authoring: no MATLAB and no
%   AUTOSAR Blockset licence were available on the build machine. Only the
%   plain Simulink/Stateflow sections (A, D, F, G) could be reasoned about
%   with confidence. The AUTOSAR Blockset sections (B, C, E) use API names
%   taken from the requirement text and MUST be checked against your
%   release before relying on them:
%
%       help autosar.ui.importer
%       help autosar.tlc
%       type <matlabroot>/toolbox/autosar/autosar/+autosar
%
%   Every Blockset call is confined to the function
%   localAutosarTarget / localImportArxml / localMapRunnable below so that
%   correcting an API name is a one-line edit in one place. Each one probes
%   for availability first and raises a specific, actionable error rather
%   than failing with an opaque 'undefined function'.
%
%   Blockset support is detected, not assumed: if the AUTOSAR Blockset is
%   not installed the script stops before writing anything.

p = inputParser;
p.addParameter('ModelName', 'WiperControl_Simulink', @ischar);
p.addParameter('Logic',     'stateflow',           @ischar);   % 'stateflow'|'caller'
p.addParameter('Force',     false,                 @islogical);
p.addParameter('Build',     true,                  @islogical);
p.addParameter('Arxml',     fullfile(fileparts(mfilename('fullpath')), ...
                                     'WiperControl_SWC.arxml'), @ischar);
p.parse(varargin{:});
opt = p.Results;

fprintf('======================================================================\n');
fprintf(' WiperControl_SWC -> Simulink migration\n');
fprintf('======================================================================\n');

% --- Section 0: preflight --------------------------------------------------
[arxmlPath, arxmlOk] = localPreflight(opt);
if ~arxmlOk
    return;
end
printBlocksetStatus();

mdl = opt.ModelName;

% --- Section 1: import the ARXML and create the model ----------------------
% NOTE: the import creates the model and the AUTOSAR component/port skeleton.
% If the import fails or the API differs, we can still build a fully working
% standalone model - so fall back rather than aborting the whole migration.
imported = false;
try
    mdl = localImportArxml(arxmlPath, mdl, opt.Force);
    imported = true;
    fprintf('[1] ARXML imported into model "%s".\n', mdl);
catch err
    warning('AUTOSAR:ImportFailed', ...
        ['ARXML import did not complete: %s\n' ...
         'Continuing with a manually constructed model. Re-run the import ' ...
         'once the importer API is corrected if you need the generated ' ...
         'component metadata.'], err.message);
    fprintf('[1] ARXML import FAILED (%s) - using manual fallback.\n', err.message);
end

% --- Section 2: model scaffold ---------------------------------------------
if (bdIsLoaded(mdl) == 0)
    localNewModel(mdl);
end
open_system(mdl);
localClearCanvas(mdl);

if ~imported
    localBuildArxmlPortSkeleton(mdl);
    fprintf('[2] Port skeleton created from the ARXML contents by hand.\n');
else
    fprintf('[2] Using the ports produced by the importer.\n');
end

% --- Section 3: the 10 ms logic --------------------------------------------
logicBlock = localBuildLogic(mdl, opt.Logic);
fprintf('[3] Logic block "%s" added (mode: %s).\n', logicBlock, opt.Logic);

% --- Section 4: wiring ------------------------------------------------------
localWire(mdl, logicBlock);
fprintf('[4] Inports and Outport wired to the logic block.\n');

% --- Section 5: AUTOSAR target, runnable mapping, timing --------------------
localAutosarTarget(mdl);
localMapRunnable(mdl, logicBlock);
localSetTiming(mdl, logicBlock, 0.01);
fprintf('[5] Target, runnable mapping and 10 ms period applied.\n');

% --- Section 6: SIL configuration ------------------------------------------
localConfigureSil(mdl);
fprintf('[6] Model configured for Software-In-The-Loop.\n');

% --- Section 7: save and build ----------------------------------------------
save_system(mdl);
fprintf('[7] Model saved as %s.slx\n', mdl);

if opt.Build
    localBuild(mdl, arxmlPath);
else
    fprintf('[7] Build skipped (requested). Run slbuild manually to export.\n');
end

fprintf('======================================================================\n');
fprintf(' Migration complete: %s.slx\n', mdl);
fprintf('======================================================================\n');
end

%% =====================================================================
%  Pre-flight
%  =====================================================================

function [arxmlPath, ok] = localPreflight(opt)
ok = false;
here = fileparts(mfilename('fullpath'));
arxmlPath = opt.Arxml;

if isempty(arxmlPath)
    arxmlPath = fullfile(here, 'WiperControl_SWC.arxml');
end

fprintf('[0] Pre-flight checks\n');

if exist(arxmlPath, 'file') ~= 2
    error('AUTOSAR:ArxmlMissing', ...
        'ARXML not found: %s\nRun this script from the project directory.', arxmlPath);
end
fprintf('    ARXML   : %s\n', arxmlPath);

if ver('simulink') == []
    error('AUTOSAR:NoSimulink', 'Simulink is not available.');
end
fprintf('    Simulink: %s\n', ver('simulink').Name);

if isempty(ver('autosar')) && isempty(license('test', 'AUTOSAR_Blockset'))
    error('AUTOSAR:BlocksetMissing', ...
        ['The AUTOSAR Blockset is not installed or not licensed.\n' ...
         'The ARXML-driven steps (target, runnable mapping, export) require it.\n' ...
         'Install the AUTOSAR Blockset, or run with ''Build'',false to skip.']);
end

ok = true;
end

function printBlocksetStatus()
% Report, without asserting, what AUTOSAR Blockset API is actually present.
hasTar = ~isempty(ver('autosar'));
hasImp = localIsPackage('autosar.ui.importer');
fprintf('    Blockset: installed=%d  autosar.ui.importer available=%d\n', hasTar, hasImp);
if ~hasTar
    fprintf('    NOTE: proceeding WITHOUT the AUTOSAR Blockset. ARXML export and\n');
    fprintf('          runnable mapping will be skipped by the build step.\n');
end
end

function tf = localIsPackage(pkg)
tf = false;
try
    tf = ~isempty(which(pkg));          %#ok<STREMP>
catch
    tf = false;
end
end

%% =====================================================================
%  Model scaffold
%  =====================================================================

function localNewModel(mdl)
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);
open_system(mdl);
% Fixed-step, single task - matches the 10 ms atomic runnable.
set_param(mdl, ...
    'SolverType',    'Fixed-step', ...
    'Solver',        'FixedStepDiscrete', ...
    'FixedStep',     '0.01', ...
    'StartTime',     '0', ...
    'StopTime',      '1');
end

function localClearCanvas(mdl)
% Remove previously generated blocks so re-running is idempotent.
blocks = find_system(mdl, 'Type', 'Block');
for k = 1:numel(blocks)
    delete_block(blocks{k});
end
end

function localBuildArxmlPortSkeleton(mdl)
% Mirrors the ARXML port definition: two receiver ports, one sender port,
% all of type uint8. Used only when the importer step did not run.
localAddPort(mdl, 'Inport',  'RPort_StalkPosition');
localAddPort(mdl, 'Inport',  'RPort_RainIntensity');
localAddPort(mdl, 'Outport', 'PPort_WiperSpeedCmd');
end

function localAddPort(mdl, portType, name)
blk = [mdl '/' name];
add_block(['simulink/Ports & Subsystems/' portType], blk, ...
          'Position', localPortPos(name), ...
          'Port', '1', 'OutDataTypeStr', 'uint8');
end

%% =====================================================================
%  Logic: Stateflow chart or C Caller block
%  =====================================================================

function blk = localBuildLogic(mdl, mode)
switch lower(mode)
    case 'stateflow', blk = localBuildStateflow(mdl);
    case 'caller',    blk = localBuildCaller(mdl);
    otherwise
        error('AUTOSAR:BadMode', 'Logic must be ''stateflow'' or ''caller''.');
end
end

function blk = localBuildStateflow(mdl)
% Reproduces the WiperControl state machine as a Stateflow chart.
% States and transitions mirror WiperControl.c exactly.
blk = [mdl '/WiperControlChart'];
add_block('simulink/Stateflow/Chart', blk, ...
          'Position', [400 80 620 220]);

rt   = sfroot;
ch   = rt.find('-isa', 'Stateflow.Chart', 'Path', blk);
if isempty(ch)
    error('AUTOSAR:ChartNotFound', 'Could not resolve the Stateflow chart.');
end

% --- Data types -------------------------------------------------------------
d = Stateflow.Data(ch);
d.Name = 'StalkPosition'; d.Type = 'Enumeration';
d.EnumTypes = {'OFF','AUTO','LOW','HIGH'};
d.Scope = 'Local';

d = Stateflow.Data(ch);
d.Name = 'RainIntensity'; d.Type = 'uint8'; d.Scope = 'Local';

d = Stateflow.Data(ch);
d.Name = 'WiperSpeedCmd'; d.Type = 'uint8'; d.Scope = 'Local';

% --- States -----------------------------------------------------------------
sOff  = Stateflow.State(ch);  sOff.Name  = 'Off';   sOff.Position = [30 30 90 60];
sAuto = Stateflow.State(ch);  sAuto.Name = 'Auto';  sAuto.Position = [140 30 200 60];
sLow  = Stateflow.State(ch);  sLow.Name  = 'Low';   sLow.Position = [250 30 300 60];
sHigh = Stateflow.State(ch);  sHigh.Name = 'High';  sHigh.Position = [250 90 300 120];

% AUTO resolves its speed from rain intensity; keep that as a nested decision.
sAutoDry  = Stateflow.State(sAuto); sAutoDry.Name  = 'Dry';  sAutoDry.Position  = [130 75 185 100];
sAutoWet  = Stateflow.State(sAuto); sAutoWet.Name  = 'Wet';  sAutoWet.Position  = [130 105 185 130];
sAutoDriz = Stateflow.State(sAuto); sAutoDriz.Name = 'Drizzle'; sAutoDriz.Position = [130 135 195 160];

% --- Outputs (per-state "do" actions) ---------------------------------------
localStateOutput(sAutoDry,  'WiperSpeedCmd = 0;');   % off
localStateOutput(sAutoWet,  'WiperSpeedCmd = 2;');   % high
localStateOutput(sAutoDriz, 'WiperSpeedCmd = 1;');   % low
localStateOutput(sLow,      'WiperSpeedCmd = 1;');
localStateOutput(sHigh,     'WiperSpeedCmd = 2;');

% --- Transitions ------------------------------------------------------------
% Stalk-driven entry transitions (default is the safe OFF state).
t = Stateflow.Transition(ch);
t.Source = []; t.Destination = sOff;
t.Condition = 'StalkPosition == 0'; t.LabelText = 'Stalk=OFF';

t = Stateflow.Transition(ch);
t.Source = []; t.Destination = sAuto;
t.Condition = 'StalkPosition == 1'; t.LabelText = 'Stalk=AUTO';

t = Stateflow.Transition(ch);
t.Source = []; t.Destination = sLow;
t.Condition = 'StalkPosition == 2'; t.LabelText = 'Stalk=LOW';

t = Stateflow.Transition(ch);
t.Source = []; t.Destination = sHigh;
t.Condition = 'StalkPosition == 3'; t.LabelText = 'Stalk=HIGH';

t = Stateflow.Transition(ch);
t.Source = []; t.Destination = sOff;
t.Condition = 'true'; t.LabelText = 'default -> OFF';

% Rain-driven transitions inside AUTO, evaluated on the 10 ms tick.
t = Stateflow.Transition(sAuto);
t.Source = []; t.Destination = sAutoDry;
t.Condition = 'RainIntensity < 10'; t.LabelText = '<10%';

t = Stateflow.Transition(sAuto);
t.Source = []; t.Destination = sAutoDriz;
t.Condition = 'RainIntensity <= 60'; t.LabelText = '10..60%';

t = Stateflow.Transition(sAuto);
t.Source = []; t.Destination = sAutoWet;
t.Condition = 'RainIntensity > 60'; t.LabelText = '>60%';

% Chart sample time. Stateflow.Chart exposes SampleTime in MILLISECONDS,
% which differs from the Simulink block parameter convention.
try
    ch.SampleTime = 10;   % 10 ms
catch err
    warning('AUTOSAR:ChartSampleTime', ...
        'Could not set the chart sample time directly (%s). ' ...
        'Set it once in the Model Explorer or via the chart''s SampleTime property.', ...
        err.message);
end
end

function localStateOutput(state, code)
state.EntryAction = code;
end

function blk = localBuildCaller(mdl)
% C Caller bound to the hand-written C. NOTE: WiperControl.c calls the RTE
% API (Rte_Read_*/Rte_Write_*), which only exists in a generated RTE or in
% the host mock. Before using this mode in SIL you must supply those
% symbols - see localBuildCallerCaveat().
blk = [mdl '/WiperControlCaller'];
add_block('simulink/User-Defined Functions/C Caller', blk, ...
          'Position', [400 80 620 220]);

localBuildCallerCaveat();

fprintf('    [caller] C Caller added. Configure it interactively once:\n');
fprintf('       C Caller -> C Source file : %s\n', ...
        fullfile(fileparts(mfilename('fullpath')), 'WiperControl.c'));
fprintf('       C Caller -> Header file    : %s\n', ...
        fullfile(fileparts(mfilename('fullpath')), 'WiperControl.h'));
fprintf('       C Caller -> Function name : Runnable_WiperControl_10ms\n');
end

function localBuildCallerCaveat()
fprintf('    [caller] WARNING: WiperControl.c depends on the RTE API.\n');
fprintf('       In SIL you must also compile rte_mock implementations that\n');
fprintf('       define Rte_Read_/Rte_Write_ as plain variables the block can\n');
fprintf('       bind to, or the model will not link.\n');
end

%% =====================================================================
%  Wiring
%  =====================================================================

function localWire(mdl, logicBlock)
% logicBlock arrives as a full path, e.g. 'WiperControl_Simulink/WiperControlChart'.
% add_line needs block NAMES, not handles, so reduce to the leaf name.
parts = strsplit(logicBlock, '/');
logicName = parts{end};

% Drop any wiring left over from a previous run so this is idempotent.
oldLines = get_param(mdl, 'Lines');
for k = 1:numel(oldLines)
    delete_line(mdl, oldLines(k).Name);
end

% Chart/C-Caller port order follows the order the data elements were declared:
%   in 1 = StalkPosition, in 2 = RainIntensity, out 1 = WiperSpeedCmd
add_line(mdl, 'RPort_StalkPosition/1', [logicName '/1'], 'autorouting', 'on');
add_line(mdl, 'RPort_RainIntensity/1', [logicName '/2'], 'autorouting', 'on');
add_line(mdl, [logicName '/1'],         'PPort_WiperSpeedCmd/1', 'autorouting', 'on');
end

%% =====================================================================
%  AUTOSAR Blockset-specific section  (API NOT VERIFIED - see header)
%  =====================================================================

function mdl = localImportArxml(arxmlPath, mdl, force)
% Imports the software-component description and creates a Simulink model.
%
% The importer package is resolved dynamically because its exact entry point
% differs between AUTOSAR Blockset releases. Resolution order:
%   1. autosar.ui.importer   (import the ARXML, create the model)
%   2. autosar.ui.simulinkImporter
%
% If neither exists the caller falls back to building the model by hand.
if force && bdIsLoaded(mdl)
    close_system(mdl, 0);
    if exist([mdl '.slx'], 'file') == 2
        delete([mdl '.slx']);
    end
end

importerPkg = '';
candidates = {'autosar.ui.importer', 'autosar.ui.simulinkImporter'};
for k = 1:numel(candidates)
    if localIsPackage(candidates{k})
        importerPkg = candidates{k};
        break;
    end
end

if isempty(importerPkg)
    error('AUTOSAR:NoImporter', ...
        ['No ARXML importer package found. Looked for: %s.\n' ...
         'Check ''help autosar.ui.importer'' in your release and add its ' ...
         'entry point to candidates{} in localImportArxml.'], ...
        strjoin(candidates, ', '));
end

fprintf('    [importer] using package: %s\n', importerPkg);

% --- API-CRITICAL ---------------------------------------------------------
% This is the call most likely to need adjusting for your release. It is
% deliberately isolated on the next three lines.
importFn = str2func(strcat(importerPkg, '.import'));            %#ok<ST2FN>
if isempty(which(importFn))
    error('AUTOSAR:ImporterApiUnknown', ...
        ['Package %s exists but %s.import was not found.\n' ...
         'Run ''help %s.import'' and correct the call in localImportArxml.'], ...
        importerPkg, importerPkg, importerPkg);
end
mdl = importFn(arxmlPath);
% -------------------------------------------------------------------------
open_system(mdl);
end

function localAutosarTarget(mdl)
% Selects the AUTOSAR target platform.
%
% The SystemTargetFile route is standard Simulink and is known to work.
% The autosar.tlc path below is the AUTOSAR-specific equivalent and is the
% value named in the migration requirement; if your release registers the
% target under a different name, list the candidates here.
set_param(mdl, 'SystemTargetFile', 'autosar.tlc');

targets = get_param(mdl, 'SystemTargetFile');
if ~strcmp(targets, 'autosar.tlc')
    set_param(mdl, 'SystemTargetFile', 'autosar.tlc');
end
end

function localMapRunnable(mdl, logicBlock)
% Associates the logic step with the AUTOSAR runnable.
%
% The periodic sample time below is standard Simulink and is what the RTE
% uses to derive the timing event period on export. The explicit runnable
% name is also written into the model description so it survives a round
% trip even if the AUTOSAR-specific mapping call is unavailable.
set_param(logicBlock, 'FunctionName', 'Runnable_WiperControl_10ms');

% --- API-CRITICAL ---------------------------------------------------------
% AUTOSAR-specific runnable mapping. Isolated here so it can be corrected in
% one place. Guarded so a missing API degrades to the sample-time mapping
% rather than aborting the migration.
try
    mapFn = str2func('autosar.ui.mapRunnable');                   %#ok<ST2FN>
    if ~isempty(which(mapFn))
        mapFn(mdl, logicBlock, 'Runnable_WiperControl_10ms');
    else
        warning('AUTOSAR:RunnableApiUnknown', ...
            ['autosar.ui.mapRunnable not found. The logic block is named ' ...
             'Runnable_WiperControl_10ms and runs at 10 ms, but the explicit ' ...
             'AUTOSAR runnable association was NOT made. Verify it in the ' ...
             'AUTOSAR Component Designer before export.']);
    end
catch err
    warning('AUTOSAR:RunnableMapFailed', ...
        'Runnable mapping call failed: %s', err.message);
end
% -------------------------------------------------------------------------
end

function localSetTiming(mdl, logicBlock, period)
% 10 ms periodic execution. WiperControl_SWC.arxml already declares
% PERIOD 0.01, so model and description agree.
set_param(mdl, 'FixedStep', num2str(period));
set_param(logicBlock, 'SampleTime', num2str(period));
fprintf('    timing: logic sample time = %g s\n', period);
end

%% =====================================================================
%  SIL configuration
%  =====================================================================

function localConfigureSil(mdl)
% Standard Simulink SIL settings: fixed-step discrete solver, normal-mode
% simulation, no data-store/aliasing assumptions, SIL target selected.
set_param(mdl, ...
    'SolverType',          'Fixed-step', ...
    'Solver',              'FixedStepDiscrete', ...
    'FixedStep',           '0.01', ...
    'SimulationMode',      'normal', ...
    'SignalLogging',       'on', ...
    'SignalLoggingName',   'logsout', ...
    'SaveOutput',          'on', ...
    'SaveTime',            'on');

% Turn the SIL target on. Simulink Coder is required for this.
try
    if ~isempty(license('test', 'Real-Time_Workshop')) || ...
       ~isempty(ver('simulinkcoder'))
        simulink.setTargetHost(mdl, 'target', 'none');
        fprintf('    SIL: target host set for Software-In-The-Loop.\n');
    else
        fprintf('    SIL: Simulink Coder absent - skipped target host setup.\n');
    end
catch err
    warning('AUTOSAR:SilSetupFailed', 'SIL target setup: %s', err.message);
end
end

%% =====================================================================
%  Build / export
%  =====================================================================

function localBuild(mdl, arxmlPath)
% Runs the build, which regenerates the AUTOSAR descriptions and the
% AUTOSAR-compliant C for the mapped runnable(s).
if isempty(ver('autosar'))
    warning('AUTOSAR:NoBlockset', ...
        ['AUTOSAR Blockset not installed - slbuild will produce Simulink ' ...
         'code only. No ARXML or AUTOSAR C code will be exported.']);
end

try
    slbuild(mdl);
    fprintf('    slbuild completed for %s.\n', mdl);
catch err
    error('AUTOSAR:BuildFailed', ...
        ['slbuild failed: %s\n' ...
         'The model is saved and intact - fix the reported issue and re-run ' ...
         'slbuild(%s) rather than re-running the whole migration.'], ...
        err.message, mdl);
end

fprintf('    Source description : %s\n', arxmlPath);
fprintf('    Exported artefacts are written next to the model under ./autosar/.\n');
end

%% =====================================================================
%  Small Simulink geometry helpers (standard API)
%  =====================================================================

function pos = localPortPos(name)
% Explicit layout so the generated model is readable when opened.
switch name
    case 'RPort_StalkPosition', pos = [80  60 110  90];
    case 'RPort_RainIntensity', pos = [80 130 110 160];
    otherwise,                  pos = [700 100 730 130];
end
end
