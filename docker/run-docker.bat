@echo off

setlocal
set "CurPath=%~dp0"
:: Remove trailing \
set "CurPath=%CurPath:~0,-1%"
:: Replace '\' by '/'
set "CurPath=%CurPath:\=/%"
:: Remove ':', C:/Users becomes C/Users
set "CurPath=%CurPath::=%"

call "%~dp0docker-id.bat"
docker run -d --rm -p 4567:4567 -p 22:22 --name %DockerContainerName% -v %CurPath%/..:/usr/src/%DockerProject% -w /usr/src/%DockerProject% %DockerUserName%/%DockerImageName%:%DockerImageTag%

rem Running "bash" command causes the entry point "user/sbin/sshd -D" not run or not run correctly

endlocal
