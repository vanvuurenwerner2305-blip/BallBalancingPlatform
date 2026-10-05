@echo off
rem Start the Ball Balancing Platform app with the "bbp" conda environment.
rem Create the environment once with:  conda env create -f app\environment.yml
setlocal
set "ENVPY="
for %%P in ("%USERPROFILE%\anaconda3\envs\bbp" "%USERPROFILE%\miniconda3\envs\bbp" "%LOCALAPPDATA%\anaconda3\envs\bbp" "%ProgramData%\anaconda3\envs\bbp") do (
  if exist "%%~P\pythonw.exe" if not defined ENVPY set "ENVPY=%%~P\pythonw.exe"
)
if not defined ENVPY (
  echo Could not find the "bbp" conda environment. Create it with:
  echo     conda env create -f "%~dp0environment.yml"
  pause
  exit /b 1
)
start "" "%ENVPY%" "%~dp0run_app.py"
