##########################################################################
# Readme.txt file for AVG2011 project

Download Acroname product sources and integrate the SVN repository. 
Aka, how do I get these files to just work with the download files. 
Launch a terminal session and get to it. 

1.) Move into a directory that you just downloaded the files from Acroname.
2.) Copy the files into your avc2011repo directory

$ cp -Rv acroname/ /Users/you/avc2011rep/source

That should be it. Get working!

##########################################################################
# TEAWare information
# Getting the Stem files ready for deployment to the Stem network

There is a make command to build the project settings for the stem network.

$ make dump

This should create a file to load all the TEAWare onto the Stem network. From the aConsole application:

brainstem> braindump avc2011_dump.dump