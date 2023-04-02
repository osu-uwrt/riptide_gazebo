import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  id: newObjs
  Layout.minimumWidth: 400
  Layout.minimumHeight: 300

  background: Rectangle {
    color: "transparent"
  }

  RowLayout {
    spacing: 4
ToolButton {
id: axe
ToolTip.text: "axe"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/axe/axe.png"
sourceSize.width: 2312
sourceSize.height: 2312
}
onClicked:{
CompObjectsPlugin.OnMode("axe")
}
}
ToolButton {
id: badge
ToolTip.text: "badge"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/badge/badge.png"
sourceSize.width: 2304
sourceSize.height: 4617
}
onClicked:{
CompObjectsPlugin.OnMode("badge")
}
}
ToolButton {
id: binBarrel
ToolTip.text: "binBarrel"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/binBarrel/binBarrel.png"
sourceSize.width: 1152
sourceSize.height: 2304
}
onClicked:{
CompObjectsPlugin.OnMode("binBarrel")
}
}
ToolButton {
id: binPhone
ToolTip.text: "binPhone"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/binPhone/binPhone.png"
sourceSize.width: 1152
sourceSize.height: 2304
}
onClicked:{
CompObjectsPlugin.OnMode("binPhone")
}
}
ToolButton {
id: bootlegger
ToolTip.text: "bootlegger"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/bootlegger/bootlegger.png"
sourceSize.width: 248
sourceSize.height: 504
}
onClicked:{
CompObjectsPlugin.OnMode("bootlegger")
}
}
ToolButton {
id: bootleggerTorpedo
ToolTip.text: "bootleggerTorpedo"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/bootleggerTorpedo/bootleggerTorpedo.png"
sourceSize.width: 188
sourceSize.height: 388
}
onClicked:{
CompObjectsPlugin.OnMode("bootleggerTorpedo")
}
}
ToolButton {
id: cash
ToolTip.text: "cash"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/cash/cash.png"
sourceSize.width: 2312
sourceSize.height: 2312
}
onClicked:{
CompObjectsPlugin.OnMode("cash")
}
}
ToolButton {
id: gman
ToolTip.text: "gman"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/gman/gman.png"
sourceSize.width: 248
sourceSize.height: 504
}
onClicked:{
CompObjectsPlugin.OnMode("gman")
}
}
ToolButton {
id: gmanTorpedo
ToolTip.text: "gmanTorpedo"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/gmanTorpedo/gmanTorpedo.png"
sourceSize.width: 193
sourceSize.height: 389
}
onClicked:{
CompObjectsPlugin.OnMode("gmanTorpedo")
}
}
ToolButton {
id: tempest
ToolTip.text: "tempest"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/tempest/tempest.png"
sourceSize.width: 256
sourceSize.height: 256
}
onClicked:{
CompObjectsPlugin.OnMode("tempest")
}
}
ToolButton {
id: tommyGun
ToolTip.text: "tommyGun"
ToolTip.visible: hovered
ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
contentItem: Image {
fillMode: Image.Stretch
horizontalAlignment: Image.AlignHCenter
verticalAlignment: Image.AlignVCenter
source: "../../../../riptide_gui/riptide_meshes/meshes/tommyGun/tommyGun.png"
sourceSize.width: 2304
sourceSize.height: 4617
}
onClicked:{
CompObjectsPlugin.OnMode("tommyGun")
}
}
    //StartButtons
    
    
  }
}