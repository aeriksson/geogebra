#!/bin/sh
# simple ./install-sh script that copies everything to the right place

# create all needed directories
./install-sh -d -m 755 /usr/share/icons/hicolor/16x16/apps /usr/share/icons/hicolor/22x22/apps /usr/share/icons/hicolor/24x24/apps /usr/share/icons/hicolor/32x32/apps /usr/share/icons/hicolor/36x36/apps /usr/share/icons/hicolor/48x48/apps /usr/share/icons/hicolor/64x64/apps /usr/share/icons/hicolor/72x72/apps /usr/share/icons/hicolor/96x96/apps /usr/share/icons/hicolor/128x128/apps /usr/share/icons/hicolor/192x192/apps /usr/share/icons/hicolor/256x256/apps /usr/share/icons/hicolor/scalable/apps
./install-sh -d -m 755 /usr/share/icons/hicolor/16x16/mimetypes /usr/share/icons/hicolor/22x22/mimetypes /usr/share/icons/hicolor/24x24/mimetypes /usr/share/icons/hicolor/32x32/mimetypes /usr/share/icons/hicolor/36x36/mimetypes /usr/share/icons/hicolor/48x48/mimetypes /usr/share/icons/hicolor/64x64/mimetypes /usr/share/icons/hicolor/72x72/mimetypes /usr/share/icons/hicolor/96x96/mimetypes /usr/share/icons/hicolor/128x128/mimetypes /usr/share/icons/hicolor/192x192/mimetypes /usr/share/icons/hicolor/256x256/mimetypes /usr/share/icons/hicolor/scalable/mimetypes

# ./install-sh files
./install-sh -m 644 *.jar /usr/share/geogebra
./install-sh -m 755 geogebra /usr/bin
./install-sh -m 644 geogebra.xml /usr/share/mime/packages
./install-sh -m 644 geogebra.desktop /usr/share/applications

./install-sh -m 644 icons/hicolor/16x16/apps/geogebra.png /usr/share/icons/hicolor/16x16/apps
./install-sh -m 644 icons/hicolor/22x22/apps/geogebra.png /usr/share/icons/hicolor/22x22/apps
./install-sh -m 644 icons/hicolor/24x24/apps/geogebra.png /usr/share/icons/hicolor/24x24/apps
./install-sh -m 644 icons/hicolor/32x32/apps/geogebra.png /usr/share/icons/hicolor/32x32/apps
./install-sh -m 644 icons/hicolor/36x36/apps/geogebra.png /usr/share/icons/hicolor/36x36/apps
./install-sh -m 644 icons/hicolor/48x48/apps/geogebra.png /usr/share/icons/hicolor/48x48/apps
./install-sh -m 644 icons/hicolor/64x64/apps/geogebra.png /usr/share/icons/hicolor/64x64/apps
./install-sh -m 644 icons/hicolor/72x72/apps/geogebra.png /usr/share/icons/hicolor/72x72/apps
./install-sh -m 644 icons/hicolor/96x96/apps/geogebra.png /usr/share/icons/hicolor/96x96/apps
./install-sh -m 644 icons/hicolor/128x128/apps/geogebra.png /usr/share/icons/hicolor/128x128/apps
./install-sh -m 644 icons/hicolor/192x192/apps/geogebra.png /usr/share/icons/hicolor/192x192/apps
./install-sh -m 644 icons/hicolor/256x256/apps/geogebra.png /usr/share/icons/hicolor/256x256/apps
./install-sh -m 644 icons/hicolor/scalable/apps/geogebra.svgz /usr/share/icons/hicolor/scalable/apps

./install-sh -m 644 icons/hicolor/16x16/mimetypes/*.* /usr/share/icons/hicolor/16x16/mimetypes
./install-sh -m 644 icons/hicolor/22x22/mimetypes/*.* /usr/share/icons/hicolor/22x22/mimetypes
./install-sh -m 644 icons/hicolor/24x24/mimetypes/*.* /usr/share/icons/hicolor/24x24/mimetypes
./install-sh -m 644 icons/hicolor/32x32/mimetypes/*.* /usr/share/icons/hicolor/32x32/mimetypes
./install-sh -m 644 icons/hicolor/36x36/mimetypes/*.* /usr/share/icons/hicolor/36x36/mimetypes
./install-sh -m 644 icons/hicolor/48x48/mimetypes/*.* /usr/share/icons/hicolor/48x48/mimetypes
./install-sh -m 644 icons/hicolor/64x64/mimetypes/*.* /usr/share/icons/hicolor/64x64/mimetypes
./install-sh -m 644 icons/hicolor/72x72/mimetypes/*.* /usr/share/icons/hicolor/72x72/mimetypes
./install-sh -m 644 icons/hicolor/96x96/mimetypes/*.* /usr/share/icons/hicolor/96x96/mimetypes
./install-sh -m 644 icons/hicolor/128x128/mimetypes/*.* /usr/share/icons/hicolor/128x128/mimetypes
./install-sh -m 644 icons/hicolor/192x192/mimetypes/*.* /usr/share/icons/hicolor/192x192/mimetypes
./install-sh -m 644 icons/hicolor/256x256/mimetypes/*.* /usr/share/icons/hicolor/256x256/mimetypes
./install-sh -m 644 icons/hicolor/scalable/mimetypes/*.* /usr/share/icons/hicolor/scalable/mimetypes
