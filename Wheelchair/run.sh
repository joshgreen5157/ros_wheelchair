ServerNum="1"
Container_Name="autowheelchair"
Container_instance="autowheelchair-$ServerNum"
Display="10.0.0.13:0.0"

sh buildDocker.sh $Container_Name> /dev/null

docker run -d -t --rm --name autowheelchair -e DISPLAY=$Display $Container_Name