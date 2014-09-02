^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package neo_watchdogs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2014-09-02)
------------------
* fixed .temperature to .data for std_msgs::Int16
* --> removed
* neo_watchdogs updated message files from pr2_msgs to neo_msgs
* pr2 dep removed
* removed pr2_msgs from CMakeFiles
* pr2_msgs removed from build and run dependecies
* Contributors: Osiron007

0.1.1 (2014-09-01)
------------------
* package.xml files version, license and maintainer checked
* new version using catkin and replacing old version
* replaced neo_msgs with cob_relayboard
* neo_watchdogs: changed default topic names
* changed relayboards battery msg to pr2s powerstate
* restructured packages: central neo_msgs pkg
* added watchdog nodes for undervoltage / temperature / error stop
* Contributors: Osiron007, Timo Hackel
