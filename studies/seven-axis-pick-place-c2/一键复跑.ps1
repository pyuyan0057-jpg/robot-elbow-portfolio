param(
    [ValidateSet('both','ball','cylinder')][string]$Task = 'both',
    [ValidateSet('ros','physics')][string]$Mode = 'ros',
    [string]$Solver = $env:ARM_SOLVER_PYTHON,
    [string]$Runtime = $env:ARM_ROS_RUNTIME
)
$ErrorActionPreference = 'Stop'
if (-not $Solver) {
    $Solver = (Get-Command python).Source
}
$env:PYTHONUTF8 = '1'
if (-not (Test-Path -LiteralPath "$PSScriptRoot/dynamics.dll")) {
    & $Solver "$PSScriptRoot/build_runtime.py"
    if ($LASTEXITCODE) { throw 'Dynamics library compilation failed.' }
}
if ($Mode -eq 'physics') {
    & $Solver "$PSScriptRoot/reproduce.py" --task $Task
    exit $LASTEXITCODE
}
$pixi=(Get-Command pixi).Source
$manifest=Join-Path $PSScriptRoot 'pixi.toml'
$jobs = if ($Task -eq 'both') { @('ball','cylinder') } else { @($Task) }
foreach ($object in $jobs) {
    & $pixi run --manifest-path $manifest python "$PSScriptRoot/ros_driver.py" $Solver $object
    if ($LASTEXITCODE) { throw "ROS trial failed for $object" }
    $result = Get-Content -Raw -LiteralPath "$PSScriptRoot/final_$object.json" | ConvertFrom-Json
    if (-not $result.passed) { throw "Task acceptance failed for $object" }
    Get-Content -Raw -LiteralPath "$PSScriptRoot/${object}_ros_verify.json"
}
