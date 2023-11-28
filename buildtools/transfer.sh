


TRANSFER_POWERSHELL="scp -r \\wsl.localhost\Ubuntu-22.04\home\berke\autopilot\buildtools\build\$PLATFORM autopilot@autopilot.local:/home/autopilot/autopilot\buildtools\build"
DEPLOY_POWERSHELL="ssh autopilot@autopilot.local 'bash -s < /home/autopilot/autopilot/buildtools/deploy.sh --no-unpack'"

powershell.exe -Command $TRANSFER_POWERSHELL
powershell.exe -Command $DEPLOY_POWERSHELL
