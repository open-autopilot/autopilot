SCRIPT_DIR="$(dirname "$0")"
BUILD_DIR="$SCRIPT_DIR/build"
ASSETS_DIR="$SCRIPT_DIR/assets"

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        # Service option
        -s | --service)
            SERVICES="$2"
            shift
            ;;
        -n | --no-unpack)
            NO_UNPACK=true
            ;;
        *)

        # Unknown option
        echo "Unknown option: $key"
        exit 1
        ;;
    esac
    shift
done

if [$NO_UNPACK]; then  
    CONTAINERFILES=($(find "$BUILD_DIR" -type f -name "*.tar"))
    HOST_PLATFORM=$(docker version --format '{{.Client.Os}}/{{.Client.Arch}}')
    HOST_PLATFORM_CLEANED=$(echo "$HOST_PLATFORM" | tr '/' '-')

    CONTAINER_COUNT=0
    for FILE in "${CONTAINERFILES[@]}"; do
        CONTAINERFILE_NAME=$(basename "$FILE")
        CONTAINERFILE_NO_EXT=$(echo "$CONTAINERFILE_NAME" | cut -f 1 -d '.')
        CONTAINERFILE_PLATFORM=$(echo "$CONTAINERFILE_NAME" | cut -f 2 -d '.')

        # Check if $SERVICE is equal to basename or if $SERVICE is null
        if [ "$SERVICES" = "$CONTAINERFILE_NO_EXT" ] || [[ -z "$SERVICES" ]]; then
            if [ "$(stat -c%s $FILE)" -gt 10 ] && [ "$CONTAINERFILE_PLATFORM" = "$HOST_PLATFORM_CLEANED" ]; then
                echo "Loading container file: $FILE"
                docker rmi $CONTAINERFILE_NO_EXT:latest > /dev/null 2>&1
                docker import $FILE $CONTAINERFILE_NO_EXT:latest
                ((COUNTER++))
            fi
        fi
    done
fi 

if [[ -z "$SERVICES" ]]; then
    echo "Deploying all containers"
    docker compose -f $ASSETS_DIR/docker-compose.yml up -d --force-recreate
else 
    echo "Deploying container: $SERVICES"
    docker compose -f $ASSETS_DIR/docker-compose.yml up -d --force-recreate $SERVICES
fi

