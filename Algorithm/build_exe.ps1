<#
Builds simulator.exe and send_to_car.exe with PyInstaller and copies
config.yaml alongside them in dist/. Run from the Algorithm/ directory:

    .\build_exe.ps1

Requires: pip install -r requirements-build.txt (and requirements.txt)

If PyInstaller reports it doesn't support the active Python version,
rebuild inside a Python 3.11/3.12 venv instead — this only affects the
machine doing the build, not teammates running the committed .exe.

--paths . puts Algorithm/ on PyInstaller's import search path, which
simulator/main.py needs since it does `from simulator.arena import ...`
(an absolute import) from *inside* the simulator package.
#>

$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot

pyinstaller --onefile --distpath dist --workpath build --specpath build --paths . --name simulator simulator/main.py

pyinstaller --onefile --distpath dist --workpath build --specpath build --paths . --name send_to_car send_to_car.py

Copy-Item -Path "config.yaml" -Destination "dist\config.yaml" -Force

Write-Host "Build complete: dist\simulator.exe, dist\send_to_car.exe, dist\config.yaml"
