# CMS and Canvas Mode Guide

Starting with Betaflight v3.1.0, flight controller (FC) will have a Configuration Menu System that operates across number of different display devices, such as FC-integrated OSD, external OSD with CANVAS extension, OLED display and radio telemetry screen. MWOSD supports the CMS with CANVAS extension.

## FC configuration

FC firmware must be built with `CMS` and `CANVAS` or equivalent options, and corresponding features should be turned on in configuration if they are controlled via features.

## MWOSD configuration

In `Config.h`, `CANVAS_SUPPORT` option must be enabled.

`BETAFLIGHT` option will do this automatically (for v3.1.0 as of 2016-10-26).

## Caveats

### Dual CMS chaos

When the FC-side CMS is enabled, you are going to have two CMS in your system; one in the FC and one in MWOSD. CMS invocation stick commands should be different, but you must be aware that these two CMS may not have the same navigational model and command system. You have to deal with this duality.

One thing you absolutely have to avoid is to enter FC-side CMS invocation command while you are in the MWOSD CMS. While commands may work on MWOSD CMS, those commands are also handled in FC-side CMS and may produce unexpected results. (If you successfully exit the MWOSD CMS, you will end up in the OOS state explained below.)

### OOS (Out-Of-Sync)

MWOSD is very stable, and so is the canvas mode support. 

However, since the canvas mode protocol is simplex from FC to MWOSD, CMS on FC and MWOSD may get out-of-sync, and your stick inputs may not be reflected to what is displayed on the screen.

- Resetting or power cycling the MWOSD while the CMS is active will induce the OOS state at 120% probability.
- Entering FC-side CMS invocation command while in the MWOSD-side CMS will very likely to end up in the OOS.
- Other cases are not known yet.

You can tell the out-of-sync state by:

1. For the above mentioned reset case, the MWOSD will not get out of opening screen.
2. You may see an asterisk character ('*') at upper left corner of your screen when this happens.
3. You may also see cursor character move as you input navigational stick commands.
4. Other erratic text displayed (not a screen full of random characters).

There are numbers of ways to get out of this state.

1. Enter a stick command that causes page redraw, such as menu back. (It is not a wise move to enter a stick command that causes item selection.)
2. Blindly navigate to BACK or EXIT menu item and select it.
3. Reset or power cycle your flight controller.

Happy Flying!
