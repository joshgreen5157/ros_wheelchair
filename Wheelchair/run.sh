ServerNum="1"
Container_Name="4800_wheelchair"
Container_instance="4800_wheelchair-$ServerNum"

sh buildDocker.sh $Container_Name> /dev/null

docker run -d -t --rm $Container_Name