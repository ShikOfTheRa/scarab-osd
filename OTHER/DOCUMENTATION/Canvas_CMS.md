# CMS and Canvas Mode Guide

Starting with Betaflight v3.1.0, flight controller (FC) will have a Configuration Menu System that operates across number of different display devices, such as FC-integrated OSD, external OSD that understands new MSP_DISPLAYPORT message, OLED display and radio telemetry screen. MWOSD supports the CMS with CANVAS extension.

## FC configuration

FC firmware must be built with `CMS` and `USE_MSP_DISPLAYPORT` or equivalent options, and corresponding features should be turned on in configuration if they are controlled via features.

BetaFlight v3.1.0 for targets F3, F4 and F7 (except for those with FC-integrated-SPI-connected OSD) all have these turned on, and ready for CMS running on top of MWOSD.


## MWOSD configuration

In `Config.h`, `CANVAS_SUPPORT` option must be enabled.

`BETAFLIGHT` option, will do this automatically (for v3.1.0 as of 2016-10-26).

## Caveats

### Dual Menu Chaos

When the FC-side CMS is enabled, you are going to have two menu systems; one in the FC and one in MWOSD. It would certainly an awful experience if you activate both menus simultaneously.

MWOSD has two measures built-in to avoid the dual menu chaos:

1. MWOSD's menu will not be invoked if FC-side CMS is active.

2. MWOSD's menu will QUIT immediately

Even with these measures, you must still be careful not to activate two menu systems simultaneously.


### OOS (Out-Of-Sync)

MWOSD is very stable, and so is the canvas mode support. 

However, since the canvas mode protocol is simplex from FC to MWOSD, CMS on FC and MWOSD may get out-of-sync in a rare case, such as resetting or power cycling the MWOSD while the CMS is active.

You can tell the out-of-sync state by:

1. For the above mentioned reset case, the MWOSD may not get out of opening screen.
2. You may see an asterisk character ('*') at upper left corner of your screen when this happens.
3. You may also see cursor character move as you input navigational stick commands.
4. Other erratic text displayed (not a screen full of random characters).

There are numbers of ways to get out of this state.

1. Enter a stick command that causes page redraw, such as menu back. (It is not a wise move to enter a stick command that causes item selection.)
2. Blindly navigate to BACK or EXIT menu item and select it.
3. Reset or power cycle your flight controller.

Happy Flying!
