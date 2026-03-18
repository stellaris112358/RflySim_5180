@ECHO OFF

REM Run script as administrator
%1 mshta vbscript:CreateObject("Shell.Application").ShellExecute("cmd.exe","/c %~s0 ::","","runas",1)(window.close)&&exit


REM The text start with 'REM' is annotation, the following options are corresponding to Options on CopterSim

REM Set the path of the RflySim tools
SET PSP_PATH=E:\PX4PSP
SET PSP_PATH_LINUX=/mnt/e/PX4PSP
E:

REM Start index of vehicle number (should larger than 0)
SET /a START_INDEX=1

REM Set the vehicleType/ClassID of vehicle 3D display in RflySim3D
SET /a CLASS_3D_ID=-1

REM Set use DLL model name or not, use number index or name string
set DLLModel=0

REM Check if DLLModel is a name string, if yes, copy the DLL file to CopterSim folder
SET /A DLLModelVal=DLLModel
if %DLLModelVal% NEQ %DLLModel% (
    REM Copy the latest dll file to CopterSim folder
    copy /Y "%~dp0"\%DLLModel%.dll %PSP_PATH%\CopterSim\external\model\%DLLModel%.dll
)

REM Set the simulation mode on CopterSim, use number index or name string
set SimMode=2

REM Set the vehicle-model (airframe) of PX4 SITL simulation
set PX4SitlFrame=iris


REM Set the map, use index or name of the map on CopterSim
SET UE4_MAP=lab_map_dynamic
@REM SET UE4_MAP=pursuer_and_evader_s

REM Set the origin x,y position (m) and yaw angle (degree) at the map
REM Position matches the second drone in swarm simulation
SET /a ORIGIN_POS_X=0
SET /a ORIGIN_POS_Y=-8
SET /a ORIGIN_YAW=0

REM Set the interval between two vehicle, unit:m
SET /a VEHICLE_INTERVAL=2


REM Set broadcast to other computer
SET IS_BROADCAST=0

REM Set UDP data mode
SET UDPSIMMODE=2


ECHO.
ECHO ---------------------------------------
ECHO Starting Single Drone Simulation (Vehicle ID: %START_INDEX%)
ECHO ---------------------------------------
ECHO.

REM QGCPath
tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe -noComPix
ECHO Start QGroundControl

REM UE4Path
cd %PSP_PATH%\RflySim3D
tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe
choice /t 5 /d y /n >nul


tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
ECHO Kill all CopterSims


REM CptSmPath
cd %PSP_PATH%\CopterSim

REM Calculate position for the selected drone (use START_INDEX so it matches swarm first drone)
set /a PosXX=(%START_INDEX%-1)*%VEHICLE_INTERVAL% + %ORIGIN_POS_X%
set /a PosYY=%ORIGIN_POS_Y%

ECHO Starting drone at position: X=%PosXX%, Y=%PosYY%
start /realtime CopterSim.exe 1 %START_INDEX% %CLASS_3D_ID% %DLLModel% %SimMode% %UE4_MAP% %IS_BROADCAST% %PosXX% %PosYY% %ORIGIN_YAW% 1 %UDPSIMMODE%
choice /t 1 /d y /n >nul

REM Set ToolChainType 1:Win10WSL 3:Cygwin
SET /a ToolChainType=1

if "%IS_BROADCAST%" == "0" (
    SET IS_BROADCAST=0
) else (
    SET IS_BROADCAST=1
)

SET WINDOWSPATH=%PATH%
if %ToolChainType% EQU 1 (
    wsl echo Starting PX4 Build; cd %PSP_PATH_LINUX%/Firmware; ./BkFile/EnvOri.sh; export PATH=$HOME/ninja:$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:$PATH;make px4_sitl_default; ./Tools/sitl_multiple_run_rfly.sh 1 %START_INDEX% %PX4SitlFrame%;echo Press any key to exit; read -n 1
) else (
    REM CYGPath
    cd %PSP_PATH%\CygwinToolchain
    CALL setPX4Env.bat
    bash -l -i -c 'echo Starting SITL SIMULATION; cd %PSP_PATH_LINUX%/Firmware; ./BkFile/EnvOri.sh; pwd; make px4_sitl_default; ./Tools/sitl_multiple_run_rfly.sh 1 %START_INDEX% %PX4SitlFrame%;echo Press any key to exit; read -n 1; pkill -x px4 || true;'
)
SET PATH=%WINDOWSPATH%


REM kill all applications when press a key
tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"
tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"

ECHO Simulation End.