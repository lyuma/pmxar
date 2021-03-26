#!/bin/bash

git submodule update --init

cd submodules/bullet3/build3
./premake4_osx xcode4

INTERESTING_PROJECTS=xcode4/*.xcodeproj/project.pbxproj
for proj in $INTERESTING_PROJECTS; do
    sed -i '.bak' -e 's/..\/..\/bin/..\/..\/macbin/g' $proj
    echo "Set default arch and ios sdk on $proj"
done

mkdir -p submodules/bullet3/macbin

./premake4_osx --ios xcode4

#INTERESTING_PROJECTS="xcode4ios/BulletCollision.xcodeproj/project.pbxproj xcode4ios/BulletDynamics.xcodeproj/project.pbxproj xcode4ios/LinearMath.xcodeproj/project.pbxproj xcode4ios/BussIK.xcodeproj/project.pbxproj"
INTERESTING_PROJECTS=xcode4ios/*.xcodeproj/project.pbxproj
for proj in $INTERESTING_PROJECTS; do
    sed -i '.bak' -e 's/ = macos[a-z]*/ = iphoneos/' -e '/ =.*x86_64/d' -e '/ =.*armv7/d' -e '/OBJROOT =/d' -e '/SYMROOT =/d' $proj
    echo "Set default arch and ios sdk on $proj"
done
