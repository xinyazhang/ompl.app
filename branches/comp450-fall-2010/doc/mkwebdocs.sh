#!/bin/sh

# web server
export SERVER=arachne.cs.rice.edu
# directory on web server where different versions of the web site 
# and other assets are located.
export ASSETS_ROOT=/mnt/data2/ompl
export ASSET_DIR=beta

rm -rf ${ASSET_DIR}
# copy all assets to the ASSET_DIR directory
mkdir -p ${ASSET_DIR}
for f in html/*.html; do
	sed 's/="..\//=".\//g' $f > ${ASSET_DIR}/`basename $f`
done
cp -r css js images html/*.png html/*.map ${ASSET_DIR}
chmod -R a+rX ${ASSET_DIR}

# copy everything to web server
tar cf - --exclude .svn ${ASSET_DIR} | ssh ${SERVER} 'cd '${ASSETS_ROOT}'; tar xf -'

# clean up
rm -rf ${ASSET_DIR}

echo The web site has been copied to:
echo "      " ${SERVER}:${ASSETS_ROOT}/${ASSET_DIR}.
