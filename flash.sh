#! /bin/bash

# Script for device flashing/programming
MDB="/Applications/microchip/mplabx/v5.50/mplab_platform/bin/mdb.sh"
# Hex file to be programmed to device
HEX="./dist/*/production/qontrol_firmware.X.production.hex"
# Temp file to store instructions to script
TEMP='temp.txt'
# Log file
LOG='log.txt'
# Device part number
DEVICE='PIC24FV32KA304'
# Programmer
# PROGRAMMER='pickit3'
PROGRAMMER='icd4'
# PROGRAMMER='pickit4'



# Cd to current directory
DIR=$(dirname "$0")
cd "$DIR"

# Check if we have our MPlab script file
if [ ! -f $MDB ]; then
    echo "MplabX script file not found...ABORT"
    exit
fi

# Delete any previous compilations
if [ -d ./dist ]; then
    echo "  Removing old compilations"
    rm -r ./dist
fi

# Setup configuration

# Check if a certain target was requested
if [ "$#" -eq 1 ]; then
    
    # Get current src branch
    cd ./src
    BRANCH0=$(git branch --show-current)
    
    # Try to reset the git tree, that could remove local changes
    git reset --hard
    
    # Try to checkout the requested branch
    git checkout -q --no-guess $1
    BRANCH1=$(git branch --show-current)
    
    # Check that we successfully checked out the requested branch, otherwise fail
    if [ $BRANCH1 != $1 ]; then
    	echo
    	echo "Failed to checkout branch '$1'! Available branches:"
    	git branch --list
    	exit 1
    fi
      
    cd ..
    
    echo "  Switched to branch '$BRANCH1'"
fi

# Figure out what our target is from config.h
f=$(grep -m 1 '^\s*#define TARGET_.*' src/config.h)
if [[ "$f" =~ TARGET_(.*) ]]
then
	# Get first regex match group
	TARGET="${BASH_REMATCH[1]}"
	# Make lower case
	TARGET=$(echo $TARGET | tr '[:upper:]' '[:lower:]')
	echo "  Detected build target '$TARGET'"
else
	echo "  Failed to find build target in config.h"
	exit
fi

# Replace config in makefile
echo "  Building with conf '$TARGET'"
sed -i '' -e "s/^DEFAULTCONF=.*/DEFAULTCONF=$TARGET/g" nbproject/Makefile-impl.mk


# Compile the current program
#  For the first compilation, do not clean in order to speed the initial output to user
LINES=$(wc -l ./src/*.h ./src/*.c | grep -Eo '\d+ total' | grep -Eo '\d+')
echo -n "  Compiling $LINES lines of code... "
echo 'COMPILE START' > $LOG
{
	make
} >> $LOG 2>&1


olderr=''
try=0
while true
do
	
	# Check if compile succeeded
	if [ ! -z "$(grep -E '(?:\W(?:e|E)rror)' $LOG)" ]
	then
		if [ "$try" = 0 ]
		then
			echo 'FAILED'
			echo
			echo
		fi
		
		# Grep out candidate error lines from the compile log file
		#  Regexs are: lines pointing to .c or .h files; lines pointing to .o files; lines starting with a colon; lines starting with Error; lines starting with Warning.
		GERR=$(grep -Eo -e '^[a-zA-Z_]+\.(?:c|h):\d+:\d+.*$' -e '^.*\.o\(.*$' -e '^:.*$' -e '^.*(e|E)rror:.*$' -e '^.*(w|W)arning:.*$' $LOG)
		
		# Format those lines with colours, underlines, etc for readability
		# Hack to get grep input into perl (I can't be bothered to find a better solution)
		PERR=$(echo "$GERR" | perl -wln -e '
			m/([^\s]+)(.*)(error)(.+)/i and print "\033[4m","$1","\033[0m","$2","\033[1;91m","$3","\033[0m","$4"
			or
			m/([^\s]+)(.*)(warning)(.+)/i and print "\033[4m","$1","\033[0m","$2","\033[1;93m","$3","\033[0m","$4"
			or
			m/([^\s]+)(.*)(note)(.+)/i and print "\033[4m","$1","\033[0m","$2","\033[1;96m","$3","\033[0m","$4"
			or
			m/([^\s\(\)]+)(?:\(.+\))?(.*:\s\bin\b.+)/i and print "\033[4m","$1","\033[0m",": \033[1;91mlink\033[0m","$2"
			or
			m/(.*)/i and print "  ","\033[0;90m","$1","\033[0m"; ')
		
		
		if [ "$GERR" != "$olderr" ]
		then	
			echo -e "\033[41m Try ${try} \033[0m"
			echo "$PERR"
			echo
			echo
			olderr="$GERR"
			try=$((try + 1))
		fi
		
		# Compile the current program
		echo 'COMPILE' > $LOG
		# Remove previous builds
		if [ -d ./build ]; then
			rm -r ./build
		fi
		# Make
		{
			make
		} >> $LOG 2>&1
		sleep 0.1
		
	else
		echo 'DONE'
		break
	fi
done


# Check that we have our hex file
if [ ! -f $HEX ]; then
    echo "Hex file not found...ABORT"
	read -p "Press ENTER to exit."
    exit
fi

# Make HEX exact (remove wildcards etc)
HEX="$(echo $HEX)"

# Write flash script to file
{
	echo "device $DEVICE"
	echo "set system.yestoalldialog true"
	echo "set AutoSelectMemRanges auto"
# 	echo "set poweroptions.powerenable true"	# Power target
# 	echo "set voltagevalue 5.0"
	echo "set memories.eeprom false"
	echo "set programoptions.eraseb4program true"
	echo "set programoptions.pgmentry.voltage low"
	echo "hwtool $PROGRAMMER -p"
	echo "program $HEX"
	echo "quit"
} > $TEMP

# Flash device
echo -n "  Programming device $DEVICE with $PROGRAMMER... "
echo 'PROGRAM START' >> $LOG
echo "Programming with $(echo $MDB)" >> $LOG
{
	$MDB $TEMP
} >> $LOG 2>&1

# Check if programming succeeded
if [ -z "$(grep 'Program succeeded' $LOG)" ]
then
	echo 'FAILED'
	echo
	echo '  Error was:'
	cat $LOG
	echo
	exit 2
else
	echo 'DONE'
fi

# Finish up
echo "  Log written to $LOG"
# Delete temp file
# rm $TEMP
# Delete other produced garbage
#rm ./MPLABXLog.xml*

