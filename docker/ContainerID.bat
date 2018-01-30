@echo off 

set ContainerID=
docker ps -qf ancestor=%DockerImageName%:%DockerImageTag% > %TEMP%\ContainerID.txt & set /p ContainerID= < %TEMP%\ContainerID.txt & del %TEMP%\ContainerID.txt
if defined ContainerID if /i not "%ContainerID%"=="" echo %ContainerID%