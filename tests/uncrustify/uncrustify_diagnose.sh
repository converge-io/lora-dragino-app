# This script allows to check if the config converge.cfg behaves as expected by creating a diff file to check what uncrustify would have changed
OUTPUT_DIR="_output/uncrustify/"
DIGEST_FILE="digest.patch"
ROOT_DIR="$PWD/../.."
mkdir -p $ROOT_DIR/$OUTPUT_DIR
docker run -v $ROOT_DIR:/app converge/geodude-hw-app:v0.6.2 sh -c "cd /app && find . -path ./lib -prune -o -path ./_output -prune -name \"*.c\" -o -name \"*.h\" -print | uncrustify -c tests/uncrustify/converge.cfg --prefix $OUTPUT_DIR -F -"
(cd $ROOT_DIR/$OUTPUT_DIR && cp /dev/null $DIGEST_FILE && find . -name "*.c" -o -name "*.h" -exec sh -c 'diff $1 ../../$1 >> $2' _ {} $DIGEST_FILE \;)
