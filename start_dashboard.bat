@echo off
title MM-Robot BLE Dashboard Server
chcp 65001 >nul
echo ========================================================
echo   MM-ROBOT MICROMOUSE BLE DASHBOARD (LOCAL SERVER)
echo ========================================================
echo.
echo [1] Dang khoi dong Localhost server tai cong 8000...
echo [2] Dang mo trinh duyet Chrome/Edge tai http://localhost:8000/ble_dashboard.html ...
echo.
echo LUU Y:
echo - Khong dong cua so CMD nay khi dang su dung Dashboard!
echo - Trinh duyet Chrome hoac Edge se ho tro ket noi Web Bluetooth.
echo ========================================================
echo.

start "" "http://localhost:8000/ble_dashboard.html"
python -m http.server 8000
