BUILD='scarab-osd/build-atmega328'
SRC='MW_OSD'
DESTINATION='Installer'


make_one(){
    CONTROLLER=$1

    make -C $SRC CONTROLLER="-DUSE_${CONTROLLER}=1"
    [ -f $BUILD/MW_OSD_TMP.hex ] && mv $BUILD/MW_OSD_TMP.hex $DESTINATION/MW_OSD.$${CONTROLLER}.hex && rm -rf $BUILD

}


make_one 'CLEANFLIGHT' &&
make_one 'BETAFLIGHT' &&
