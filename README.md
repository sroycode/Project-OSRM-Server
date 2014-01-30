Project-OSRM-Server
===================

Sometimes on production machines running vendor-controlled OS, getting the lua parts of osrm to work is a pain.
For users, who just need osrm-routed on a server ( data is prepared elsewhere ) this may be of use

For instructions on how to compile and run OSRM, please consult the Wiki at
https://github.com/DennisOSRM/Project-OSRM/wiki

Compiling
=========

Mostly 

$ mkdir build

$ cd build

$ cmake ..

$ make


Running
=======

You need to add the path to the osrm generated files
in server.ini (see the sample)

Same as the main OSRM
https://github.com/DennisOSRM/Project-OSRM/wiki/Running-OSRM#running-the-engine

Todo
====

Sorry for the sparse documentation
