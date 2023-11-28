


TRANSFER_POWERSHELL="scp -r \\wsl.localhost\Ubuntu-22.04\home\berke\autopilot\buildtools autopilot@autopilot.local:/home/autopilot/autopilot"
powershell.exe -Command $TRANSFER_POWERSHELL

