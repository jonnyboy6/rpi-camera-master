plink.exe -ssh -P 22 pi@raspberrypi.local -pw raspberry "./clean"
pscp -P 22 -r -pw raspberry output/CameraVision.zip pi@raspberrypi.local:/home/pi/jar
plink.exe -ssh -P 22 pi@raspberrypi.local -pw raspberry "./install-jar"
PAUSE