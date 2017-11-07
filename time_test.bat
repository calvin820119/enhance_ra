echo off
setlocal EnableDelayedExpansion

set "startTime=%time: =0%"

set pwd=%~dp0
cd %pwd%

main.exe -NUM_UE 100
main.exe -NUM_UE 200
main.exe -NUM_UE 300
main.exe -NUM_UE 400
main.exe -NUM_UE 500
main.exe -NUM_UE 600
main.exe -NUM_UE 700
main.exe -NUM_UE 800
main.exe -NUM_UE 900
main.exe -NUM_UE 1000


set "endTime=%time: =0%"

rem Get elapsed time:
set "end=!endTime:%time:~8,1%=%%100)*100+1!"  &  set "start=!startTime:%time:~8,1%=%%100)*100+1!"
set /A "elap=((((10!end:%time:~2,1%=%%100)*60+1!%%100)-((((10!start:%time:~2,1%=%%100)*60+1!%%100)"

rem Convert elapsed time to HH:MM:SS:CC format:
set /A "cc=elap%%100+100,elap/=100,ss=elap%%60+100,elap/=60,mm=elap%%60+100,hh=elap/60+100"

echo ========================
echo Start:    %startTime%
echo End:      %endTime%
echo Elapsed:  %hh:~1%%time:~2,1%%mm:~1%%time:~2,1%%ss:~1%%time:~8,1%%cc:~1%

pause