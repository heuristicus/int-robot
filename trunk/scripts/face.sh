RUNCLASSPATH=dist/robotics.jar:build/libs/opencv/javacv.jar
CASCADEFILE=/data/private/robot/Downloads/OpenCV-2.4.3/data/haarcascades/haarcascade_frontalface_default.xml
SOURCEIMG=/data/private/robot/Desktop/group.png
OUTPUTIMG=/data/private/robot/Desktop/groupbox.png

java -cp $RUNCLASSPATH ex4.FaceDetectTest $CASCADEFILE $SOURCEIMG $OUTPUTIMG
