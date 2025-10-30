@ECHO OFF

REM Run script as administrator
NET SESSION >nul 2>&1 || powershell -Command "Start-Process cmd -ArgumentList '/c, ""%~f0""' -Verb RunAs" && exit /b


REM The text start with 'REM' is annotation, the following options are corresponding to Options on CopterSim

REM Set the path of the RflySim tools
if not defined PSP_PATH (
    SET PSP_PATH=C:\RflySimTool\PX4PSP
    SET PSP_PATH_LINUX=/mnt/c/RflySimTool/PX4PSP
)

echo With this SIL script, you can initialize your vehicle(s) anywhere you want
echo Please enter the x, y position (unit m) and yaw angle list with the separator ','
echo For example, PosX list:1.1,2.2 and PosY list:0,0 and Yaw:0,0 will create two vehicles.
REM Set Copters' x&y Position vector on the map
REM SET /P PosXStr=Please enter the PosX (m) list:
REM SET /P PosYStr=Please enter the PosY (m) list:
REM SET /P YawStr=Please enter the Yaw (degree) list:

SET PosXStr=0,0,0,10
SET PosYStr=1,2,3,2
SET YawStr=0,0,0,0

REM Start index of vehicle number (should larger than 0)
REM This option is useful for simulation with multi-computers
SET /a START_INDEX=1

REM Total vehicle Number to auto arrange position
REM SET /a TOTOAL_COPTER=8

REM Set the vehicleType/ClassID of vehicle 3D display in RflySim3D
SET /a CLASS_3D_ID=-1


REM Set use DLL model name or not, use number index or name string
REM This option is useful for simulation with other types of vehicles instead of multicopters
set DLLModel=0

REM Check if DLLModel is a name string, if yes, copy the DLL file to CopterSim folder
SET /A DLLModelVal=DLLModel
if %DLLModelVal% NEQ %DLLModel% (
    REM Copy the latest dll file to CopterSim folder
    copy /Y "%~dp0"\%DLLModel%.dll %PSP_PATH%\CopterSim\external\model\%DLLModel%.dll
)

REM Set the simulation mode on CopterSim, use number index or name string
REM e.g., SimMode=2 equals to  SimMode=PX4_SITL_RFLY
set SimMode=2

REM Set the vehicle-model (airframe) of PX4 SITL simulation, the default airframe is a quadcopter: iris
REM Check folder Firmware\ROMFS\px4fmu_common\init.d-posix for supported airframes (Note: You can also create your airframe file here)
REM E.g., fixed-wing aircraft: PX4SitlFrame=plane; small cars: PX4SitlFrame=rover
set PX4SitlFrame=iris

REM Set the map, use index or name of the map on CopterSim
REM e.g., UE4_MAP=1 equals to UE4_MAP=Grasslands
SET UE4_MAP=Changsha

REM Set the origin x,y position (m) and yaw angle (degree) at the map
SET /a ORIGIN_POS_X=0
SET /a ORIGIN_POS_Y=0
SET /a ORIGIN_YAW=0

REM Set the interval between two vehicle, unit:m 
SET /a VEHICLE_INTERVAL=2


REM Set broadcast to other computer; IS_BROADCAST=0: only this computer, IS_BROADCAST=1: broadcast; 
REM or use IP address to increase speed, e.g., IS_BROADCAST=192.168.3.1
REM Note: in IP mode, IS_BROADCAST=0 equals to IS_BROADCAST=127.0.0.1, IS_BROADCAST=1 equals to IS_BROADCAST=255.255.255.255
REM You can also use a IP list with seperator "," or ";" to specify IPs to send, e.g., 127.0.0.1,192.168.1.4,192.168.1.5
SET IS_BROADCAST=0

REM Set UDP data mode; 0: UDP_FULL, 1:UDP_Simple, 2: Mavlink_Full, 3: Mavlink_simple. input number or string
REM 4:Mavlink_NoSend, 5:Mavlink_NoGPS, 6:Mavlink_Vision (NoGPS and set PX4 EKF)
SET UDPSIMMODE=2


REM QGCPath
tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe -noComPix
ECHO Start QGroundControl

REM UE4Path
cd /d %PSP_PATH%\RflySim3D
tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe
choice /t 5 /d y /n >nul


tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
ECHO Kill all CopterSims


REM CptSmPath
cd /d %PSP_PATH%\CopterSim


set /a cntr=%START_INDEX%


SET string=%PosXStr%
SET stringY=%PosYStr%
SET stringYaw=%YawStr%
:MYSPLIT
    for /f "tokens=1,* delims=," %%i in ("%string%") do (
        set xPos=%%i
        set string=%%j
    )
    for /f "tokens=1,* delims=," %%i in ("%stringY%") do (
        set yPos=%%i
        set stringY=%%j
    )

    for /f "tokens=1,* delims=," %%i in ("%stringYaw%") do (
        set yawAng=%%i
        set stringYaw=%%j
    )

    REM echo start CopterSim
    start /realtime CopterSim.exe 1 %cntr% %CLASS_3D_ID% %DLLModel% %SimMode% %UE4_MAP% %IS_BROADCAST% %xPos% %yPos% %yawAng% 1 %UDPSIMMODE%
    ECHO start Copter #%cntr%
    choice /t 2 /d y /n >nul
    set /a cntr=%cntr%+1
    
    REM TIMEOUT /T 1
    if not "%string%"=="" (
        if "%stringY%"=="" (
            tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
            tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"
            tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"
            ECHO Error: The dimensions of PosXStr and PosYStr are not equal            
            pause
            goto ErrorPoint
        )
    )
if not "%string%"=="" goto MYSPLIT
set /a VehicleNum=%cntr%-%START_INDEX%

REM Set ToolChainType 1:Win10WSL 3:Cygwin
SET /a ToolChainType=1

if "%IS_BROADCAST%" == "0" (
    SET IS_BROADCAST=0
) else (
    SET IS_BROADCAST=1
)

SET WINDOWSPATH=%PATH%
if %ToolChainType% EQU 1 (
    wsl -d RflySim-20.04 echo Starting PX4 Build; cd %PSP_PATH_LINUX%/Firmware; ./BkFile/EnvOri.sh; export PATH=$HOME/ninja:$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:$PATH;make px4_sitl_default; ./Tools/sitl_multiple_run_rfly.sh %VehicleNum% %START_INDEX% %PX4SitlFrame%;echo Press any key to exit; read -n 1
) else (
    REM CYGPath
    cd /d %PSP_PATH%\CygwinToolchain
    CALL setPX4Env.bat
    bash -l -i -c 'echo Starting SITL SIMULATION; cd %PSP_PATH_LINUX%/Firmware; ./BkFile/EnvOri.sh; pwd; make px4_sitl_default; ./Tools/sitl_multiple_run_rfly.sh %VehicleNum% %START_INDEX% %PX4SitlFrame%;echo Press any key to exit; read -n 1; pkill -x px4 || true;'
)
SET PATH=%WINDOWSPATH%


REM kill all applications when press a key
tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"
tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"

ECHO Start End.
