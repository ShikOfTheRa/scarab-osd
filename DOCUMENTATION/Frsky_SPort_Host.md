# Frsky Smart Port (S.Port) Sensor Data without a Smart Port Receiver #


## First things first (you will/wont need) ##

Basically same as https://code.google.com/p/multiwii-osd/wiki/Frsky_SPort but without the receiver and this type of inverter instead:

  * **Wont need smart port receiver !**

  * **An inverter capable of 2 direction data transfer**, such as this:
http://hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=16672

(See Bottom of page for how to wire this up:)

![http://hobbyking.com/hobbyking/store/catalog/FRFUL1.jpg](http://hobbyking.com/hobbyking/store/catalog/FRFUL1.jpg)

I also believe this circuit would work, but i have not tried:

![http://i.imgur.com/n7iS5LW.png](http://i.imgur.com/n7iS5LW.png)

  * **mw patched with files new enough to provide the "SPORT\_HOST" option in config.h**:

https://code.google.com/p/multiwii-osd/source/browse/#hg%2FMWC_Patches%2Ffrsky_sport
or
https://code.google.com/p/scarab-osd/source/browse/#hg%2FMWC_Patches%2Ffrsky_sport

Currently code is just setup for, 1 sensor, the cell meter, though others can be used if setup. <- todo

## Issues / Support ##

See: http://www.multiwii.com/forum/posting.php?mode=post&f=8
Or: http://www.multiwii.com/forum/viewtopic.php?f=8&t=5164


## Wiring ##

![http://imgur.com/SZTm2YQ.jpg](http://imgur.com/SZTm2YQ.jpg)