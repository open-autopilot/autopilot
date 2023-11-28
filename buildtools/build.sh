# Get the directory of the script
SCRIPT_DIR="$(dirname "$0")"
PARENT_DIR="$SCRIPT_DIR/.."
PARENT_DIR_FULL=$(realpath "$PARENT_DIR")

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        # Service option
        -s | --service)
            SERVICES="$2"
            shift
            ;;
        --platform)
            PLATFORMS="$2"
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

# Set the default architecture if none is provided
if [[ -z "$PLATFORMS" ]]; then
    PLATFORMS="linux/amd64,linux/arm64"
fi
IFS=',' read -ra PLATFORM_ARRAY <<< "$PLATFORMS"

# Finding all docker files in the repository
DOCKERFILES=($(find "$PARENT_DIR" -type f -name "*.dockerfile" -not -path "$PARENT_DIR/buildtools/*"))

# Creating group for each Dockerfile
for FILE in "${DOCKERFILES[@]}"; do
    DOCKERFILE_NAME=$(basename "$FILE")
    DOCKERFILE_NAME_NO_EXT=$(echo "$DOCKERFILE_NAME" | cut -f 1 -d '.')

    # Check if $SERVICE is equal to basename or if $SERVICE is null
    if [ "$SERVICES" = "$DOCKERFILE_NAME_NO_EXT" ] || [[ -z "$SERVICES" ]]; then
        for PLATFORM in "${PLATFORM_ARRAY[@]}"; do
            PLATFORM_CLEANED=$(echo "$PLATFORM" | tr '/' '-')
            docker buildx build -f $FILE --platform $PLATFORM --build-arg="PROJECT_DIR=$PARENT_DIR_FULL" -o type=tar,dest="$SCRIPT_DIR/build/$DOCKERFILE_NAME_NO_EXT.$PLATFORM_CLEANED.tar" /
        done
    fi
done

