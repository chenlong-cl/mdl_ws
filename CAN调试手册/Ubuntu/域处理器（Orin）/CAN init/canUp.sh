#! /bin/bash
echo "version:1.0"
# defaule argument value
bitrate=500000
dbitrate=2000000
spiCan=mcp251xfd

# valid argument range
bitrateRange=(1000 2000000)
dbitrateRange=(500000 2000000)
spiCanRange=(mcp251xfd mcp25xxfd)

# all avaliable arguments
arg=(bitrate dbitrate spiCan)

function usage {
	echo ""
	echo "usage:"
	echo "Expamle: $0 bitrate=500000 dbitrate=2000000 spiCan=mcp251xfd"
	echo ""
}


for var in $@;
do
	#check format var=val
	arg_=(`echo $var | awk -F "=" '{print $1, $2}'`)
	if [ ${#arg_[@]} -ne 2 ];then
		echo "argument format error: $var"
		usage
		exit -1
	fi
	
	#check known argument name
	matchArg=""
	for setArg in ${arg[@]};
	do
		if [ $setArg = ${arg_[0]} ];then
			matchArg=$setArg
			break
		fi
	done
	if [ ! $matchArg ]; then
		echo "unkonwn argument: ${arg_[0]}"
		usage
		exit -1
	fi
	export "$matchArg=${arg_[1]}"
done

#//check arg val
	#check  bitrate
if [[ $bitrate -lt ${bitrateRange[0]} || $bitrate -gt ${bitrateRange[1]} ]];then
	echo "bitrate value of range: ${bitrateRange[@]}"
	exit -1
fi

	#check  bitrate
if [[ $dbitrate -lt ${dbitrateRange[0]} || $dbitrate -gt ${dbitrateRange[1]} ]];then
	echo "dbitrate value of range: ${dbitrateRange[@]}"
	exit -1
fi

	#check spiCan
isSpiCanValid=""
for var in ${spiCanRange[@]};
do
	if [ $spiCan = $var ];then
		isSpiCanValid=1
	fi
done
if [ ! $isSpiCanValid ];then
	echo "spiCan value of range: ${spiCanRange[@]}"
	exit -1
fi
#check arg val end

#_________________________________________________________________
echo "----------print arguments"
echo "bitrate = $bitrate" 
echo "dbitrate = $dbitrate"
echo "spiCan = $spiCan"
echo "----------print end"
#_________________________________________________________________

echo ""
echo "----------set can down"
for canIndex in {0..8}; 
do
	count=`ifconfig | grep -c ^can${canIndex}`
	if [ ${count} -ge 1 ];then
		sudo ip link set can${canIndex} down > /dev/null
		if [ $? -eq 0 ];then
			echo "set can${canIndex} down !!!"
		else
			echo "set can${canIndex} down !!!"
		fi
	fi
done

echo ""
echo "----------rmmoding CAN driver ..."
driversKo=(mcp251xfd mttcan can_raw can)
for ko in ${driversKo[@]};
do
	count=`lsmod | grep -c ^${ko}`
	if [ $count -ge 1 ];then
		echo "rmmod: $ko"
		sudo rmmod $ko > /dev/null
	fi
done
echo "----------rmmod CAN driver end"

echo " "
echo "----------Loading CAN driver ..."
driversKo=(can can_raw mttcan)
for ko in ${driversKo[@]};
do
	echo "modprobe: ${ko}"
	sudo modprobe ${ko} 
	if [ $? -ne 0 ];then
		echo "Please check /lib/modules/$(uname -r)/kernel/net/can/${ko}.ko"
		exit -1
	fi
done

echo "insmod: ${spiCan}"
sudo insmod ${spiCan}.ko 
if [ $? -ne 0 ];then
	echo "No $spiCan.ko in current directory, Please check file: ./${spiCan}.ko"
fi
echo "----------Loaded CAN driver end"


echo ""
echo "----------set can up"
for canIndex in {0..8}; 
do
	count=`ifconfig -a | grep -c ^can${canIndex}`
	if [ ${count} -ge 1 ];then
		#ip link set can${canIndex} up type can bitrate ${bitrate} dbitrate ${bitrate} berr-reporting on fd on > /dev/null
		sudo ip link set can${canIndex} up type can bitrate ${bitrate} dbitrate ${dbitrate} berr-reporting on fd on > /dev/null
		if [ $? -eq 0 ];then
			echo "set can${canIndex} up success!!!"
		else
			echo "set can${canIndex} up failed!!!"
		fi
	fi
done
echo "bitrate=$bitrate"
echo "dbitrate=$dbitrate"
echo "spiCan=$spiCan"
