Import("env")

print("load lpc21")

env.Replace(
    UPLOADCMD="lpc21isp -control -bin $SOURCE $UPLOAD_PORT 115200 12000"
)

