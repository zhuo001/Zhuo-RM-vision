# 深度图 SLAM 避障系统 - 快速启动脚本
# 使用方法: .\run_slam_test.ps1

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "深度图 SLAM 避障系统 - 快速启动" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host ""

# 检查虚拟环境
$venvPath = "$PSScriptRoot\.venv"
if (-Not (Test-Path $venvPath)) {
    Write-Host "[ERROR] 虚拟环境不存在: $venvPath" -ForegroundColor Red
    Write-Host "请先运行以下命令创建虚拟环境:" -ForegroundColor Yellow
    Write-Host "  python -m venv .venv" -ForegroundColor Yellow
    Write-Host "  .\.venv\Scripts\Activate.ps1" -ForegroundColor Yellow
    Write-Host "  python -m pip install numpy opencv-python" -ForegroundColor Yellow
    exit 1
}

# 检查依赖
Write-Host "[INFO] 检查依赖..." -ForegroundColor Green
$pythonPath = "$venvPath\Scripts\python.exe"

& $pythonPath -c "import cv2, numpy" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "[WARNING] 依赖缺失，正在安装..." -ForegroundColor Yellow
    & $pythonPath -m pip install numpy opencv-python
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] 依赖安装失败" -ForegroundColor Red
        exit 1
    }
}

Write-Host "[INFO] 依赖检查完成" -ForegroundColor Green
Write-Host ""

# 运行程序
Write-Host "[INFO] 启动深度SLAM避障系统..." -ForegroundColor Green
Write-Host "[INFO] 按 'q' 键退出程序" -ForegroundColor Yellow
Write-Host ""

& $pythonPath "$PSScriptRoot\depth_slam_obstacle.py"

Write-Host ""
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "程序已退出" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
