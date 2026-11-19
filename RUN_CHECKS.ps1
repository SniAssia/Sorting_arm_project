# Quick numerical verification only. Does not require MATLAB or PyBullet installation.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Push-Location $root
try {
    py tools\preflight.py
    if ($LASTEXITCODE -ne 0) { throw 'Offline source/route preflight failed.' }
    Push-Location PyBullet
    try {
        py simulate.py --verify-only --all
        if ($LASTEXITCODE -ne 0) { throw 'Command CSV verification failed.' }
        py test_urdf.py
        if ($LASTEXITCODE -ne 0) { throw 'URDF transform verification failed.' }
    } finally { Pop-Location }
    py -m unittest discover -s tests -v
    if ($LASTEXITCODE -ne 0) { throw 'Capacity controller unit tests failed.' }
    Write-Host 'PASS: offline checks and counter-unit tests passed. Refer to README.md.' -ForegroundColor Green
} finally { Pop-Location }
