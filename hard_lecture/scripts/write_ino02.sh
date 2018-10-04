echo "writing arduino"
echo "connect arduino on USB"

cd `dirname ${0}`/../platformio/ino02
platformio run

