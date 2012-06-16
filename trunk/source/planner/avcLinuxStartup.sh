###################################################
# FILENAME: /etc/init.d/avcLinuxStartup.sh
# boot script to launch avc headless chicken crap
# Wrote this tutorial a long time ago
# http://www.acroname.com/garcia/starting/aRelay.html
# cd /etc/rc5.d
# ln -s ../init.d/avcLinuxStartup.sh S95avcLinuxStartup
# This is specified in the /etc/inittab file on the computer 
# as the default runlevel.  The value "S95" was picked 
# to make sure the Acroname tools script is run last, 
# but before the login prompt.
###################################################

echo "Cluck like a rooster you crazy bastard"

# Change to avcrepo directory
cd /dos/avc2012/sounds

# Play the chicken sound
./play death_rooster.wav

exit 0
