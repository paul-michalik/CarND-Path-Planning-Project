@echo off

setlocal
set "_CurPath=%~dp0"
set "CurPath=%_CurPath:\=/%"
set "CurPath=%CurPath:~0,-1%"

call "%_CurPath%docker-id.bat
docker build --tag=%DockerUserName%/%DockerImageName%:%DockerImageTag% --file="%CurPath%/Dockerfile" "%CurPath%"
endlocal