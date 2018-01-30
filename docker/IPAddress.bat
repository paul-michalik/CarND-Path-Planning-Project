@echo off

set IPAddress=
docker inspect --format="{{ .NetworkSettings.IPAddress }}" %ContainerID% > %TEMP%\IPAddress.txt & set /p IPAddress= < %TEMP%\IPAddress.txt & del %TEMP%\IPAddress.txt
if defined IPAddress if /i not "%IPAddress%"=="" echo %IPAddress%