# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        # Service option
        -s | --service)
            SERVICE="$2"
            shift
            ;;
        --platform)
            PLATFORM="$2"
            shift
            ;;
        *)

        # Unknown option
        echo "Unknown option: $key"
        exit 1
        ;;
    esac
    shift
done

# Check if platform is null
if [ -z "$PLATFORM" ]; then
    echo "Error: --platform option is required."
    exit 1
fi
PLATFORM_CLEANED=$(echo "$PLATFORM" | tr '/' '-')

# Transfer build to companion device
if [[ -z "$SERVICE" ]]; then
    TRANSFER_POWERSHELL=$(echo "scp //wsl.localhost/Ubuntu-22.04/home/berke/autopilot/buildtools/build/\*$PLATFORM_CLEANED.tar autopilot@autopilot.local:/home/autopilot/autopilot/buildtools/build/")
else 
    TRANSFER_POWERSHELL=$(echo "scp //wsl.localhost/Ubuntu-22.04/home/berke/autopilot/buildtools/build/$SERVICE.$PLATFORM_CLEANED.tar autopilot@autopilot.local:/home/autopilot/autopilot/buildtools/build/")
fi

powershell.exe -Command $TRANSFER_POWERSHELL
