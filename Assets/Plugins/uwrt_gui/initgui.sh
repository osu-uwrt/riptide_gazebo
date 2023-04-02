#!/bin/bash
MESHESDIR=../../../../riptide_gui/riptide_meshes/meshes/
if [ ! -d "$MESHESDIR" ]; then
    echo "Object Mesh Directory ${MESHESDIR} does not exist!"
else
    cp -fr template/CompObjectsTemplate.cc ./CompObjects.cc
    cp -fr template/CompObjectsTemplate.hh ./CompObjects.hh
    cp -fr template/CompObjectsPluginTemplate.qml ./CompObjectsPlugin.qml
    cp -fr template/CompObjectsPluginTemplate.qrc ./CompObjectsPlugin.qrc
    for dir in ${MESHESDIR}*; do
        [ -L "${dir%/}" ] && continue
        [ ! -d "$dir" ] && continue

        NAME=${dir%%+(/)}
        NAME=${NAME##*/}
        NAME=${NAME:-/}

        for i in ${dir}/*.[pP][nN][gG]; do
            mv ${i} ${dir}/${NAME}.png
        done
        #If there are no .png files in the directory, copy in pikachu.png and rename it to name.png
        if [ ! -f ${dir}/*.[pP][nN][gG] ]; then
            cp pikachu.png ${dir}/${NAME}.png
        fi
        
        echo "Adding $NAME"
        #Check if the object png's extension is .PNG and replace it with .png
        if [ -f "${dir}/${NAME}.PNG" ]; then
            mv "${dir}/${NAME}.PNG" "${dir}/${NAME}.png"
        fi
        #Replace the name of any png file in the dir with the name of the object
        #Run grep -n -m 1 "//StartSdfDef" ./CompObjects.cc |sed  's/\([0-9]*\).*/\1/' and save the output to a variable
        SDFLINE=$(grep -n -m 1 "//StartSdfDef" ./CompObjects.cc |sed  's/\([0-9]*\).*/\1/')
        CASELINE=$(grep -n -m 1 "//StartCases" ./CompObjects.cc |sed  's/\([0-9]*\).*/\1/')
        IFELSELINE=$(grep -n -m 1 "//StartGetCompObject" ./CompObjects.cc |sed  's/\([0-9]*\).*/\1/')

        #Adding the object to the source file
        sed -i ${IFELSELINE}'iif(type=="'${NAME}'") return getCompObjectshape(CompObjectshape::'${NAME}Sdf');' ./CompObjects.cc
        sed -i ${CASELINE}'icase CompObjectshape::'${NAME}'Sdf: return '${NAME}'Sdf;' ./CompObjects.cc
        sed -i ${SDFLINE}'iconstexpr const char * '${NAME}'Sdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="'${NAME}'"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:'${dir}'/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:'${dir}'/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";' ./CompObjects.cc   
        
        #Adding the name of the object to the header file
        OBJNAMELINE=$(grep -n -m 1 "//StartNamedObjects" ./CompObjects.hh |sed  's/\([0-9]*\).*/\1/')
        sed -i ${OBJNAMELINE}'i'${NAME}'Sdf,' ./CompObjects.hh

        #Adding the name of the object image to the qrc file
        QRCIMAGELINE=$(grep -n -m 1 "<!--StartImages-->" ./CompObjectsPlugin.qrc |sed  's/\([0-9]*\).*/\1/')
        sed -i ${QRCIMAGELINE}'i        <file>'${dir}'/'${NAME}'.png</file>' ./CompObjectsPlugin.qrc


        #Adding the button to the qml file
        QMLBUTTONLINE=$(grep -n -m 1 "//StartButtons" ./CompObjectsPlugin.qml |sed  's/\([0-9]*\).*/\1/')

        sed -i ${QMLBUTTONLINE}'i}' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i   }' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i       CompObjectsPlugin.OnMode("'${NAME}'")' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i    onClicked:{' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i   }' ./CompObjectsPlugin.qml
        W=`identify ${dir}/${NAME}.png | cut -f 3 -d " " | sed s/x.*//` #width
        H=`identify ${dir}/${NAME}.png | cut -f 3 -d " " | sed s/.*x//` #height
        sed -i ${QMLBUTTONLINE}'i       sourceSize.height: '$H ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i       sourceSize.width: '$W ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i       source: "'${dir}'/'${NAME}'.png"' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i       verticalAlignment: Image.AlignVCenter' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i       horizontalAlignment: Image.AlignHCenter' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i       fillMode: Image.Stretch' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i    contentItem: Image {' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i   ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i   ToolTip.visible: hovered' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i   ToolTip.text: "'${NAME}'"' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i   id: '${NAME}'' ./CompObjectsPlugin.qml
        sed -i ${QMLBUTTONLINE}'i ToolButton {' ./CompObjectsPlugin.qml
        
    

    
    
    done
    colcon build
fi
