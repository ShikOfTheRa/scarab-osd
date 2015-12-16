#!/bin/bash

# run me to setup a mac for development in processing

pushd /tmp

# these versions are locked at known-good releases

# install processing
curl -OL https://github.com/processing/processing/releases/download/processing-0227-2.2.1/processing-2.2.1-macosx.zip
unzip processing-2.2.1-macosx.zip
mv Processing.app /Applications/
rm -rf processing-2.2.1*
rm -f Processing.app

# install controlp5
curl -O https://controlp5.googlecode.com/files/controlP5-2.1.0.zip
unzip controlP5-2.1.0.zip
mkdir -p ~/Documents/Processing/libraries/
mv controlP5 ~/Documents/Processing/libraries/
rm -rf controlP5*

# run it
open /Applications/Processing.app

popd
