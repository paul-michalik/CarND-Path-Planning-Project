@echo off

setlocal
set "_CurPath=%~dp0"
set "CurPath=%_CurPath:\=/%"
set "CurPath=%CurPath:~0,-1%"

call "%_CurPath%docker-id.bat
docker rm --force %DockerContainerName%

endlocal