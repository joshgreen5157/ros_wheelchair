ServerNum="1"
Container_Name="autowheelchair"
Container_instance="autowheelchair-$ServerNum"

sh buildDocker.sh $Container_Name> /dev/null

docker run -d -t --rm $Container_Name