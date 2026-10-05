# Create "Ball Balancing Platform" shortcuts on the desktop and in the Start menu.
#   powershell -ExecutionPolicy Bypass -File tools\install_shortcuts.ps1
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$candidates = @(
    "$env:USERPROFILE\anaconda3\envs\bbp\pythonw.exe",
    "$env:USERPROFILE\miniconda3\envs\bbp\pythonw.exe",
    "$env:LOCALAPPDATA\anaconda3\envs\bbp\pythonw.exe",
    "$env:ProgramData\anaconda3\envs\bbp\pythonw.exe"
)
$python = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $python) { throw 'The "bbp" conda environment was not found. Create it with: conda env create -f app\environment.yml' }

$targets = @(
    [Environment]::GetFolderPath('Desktop'),
    (Join-Path ([Environment]::GetFolderPath('Programs')) '')
)
$shell = New-Object -ComObject WScript.Shell
foreach ($dir in $targets) {
    $lnk = $shell.CreateShortcut((Join-Path $dir 'Ball Balancing Platform.lnk'))
    $lnk.TargetPath = $python
    $lnk.Arguments = '"' + (Join-Path $repo 'app\run_app.py') + '"'
    $lnk.WorkingDirectory = $repo
    $lnk.IconLocation = (Join-Path $repo 'app\assets\bbp.ico') + ',0'
    $lnk.Description = 'Ball Balancing Platform - vision and control simulator'
    $lnk.Save()
    Write-Output "Created $($lnk.FullName)"
}
