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
      id: o1
      ToolTip.text: "Object 1"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "pikachu.png"
        sourceSize.width: 48;
        sourceSize.height: 48;
      }
      onClicked: {
        CompObjectsPlugin.OnMode("box")
      }
    }
    ToolButton{
      id: o2
      ToolTip.text: "Object 2"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "pikachu.png"
        sourceSize.width: 48;
        sourceSize.height: 48;
      }
      onClicked: {
        CompObjectsPlugin.OnMode("sphere")
      }
    }
    ToolButton {
      id: o3
      ToolTip.text: "Object 3"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "pikachu.png"
        sourceSize.width: 48;
        sourceSize.height: 48;
      }
      onClicked: {
        CompObjectsPlugin.OnMode("cylinder")
      }
    }
    ToolButton {
      id: o4
      ToolTip.text: "Object 4"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "pikachu.png"
        sourceSize.width: 48;
        sourceSize.height: 48;
      }
      onClicked: {
        CompObjectsPlugin.OnMode("capsule")
      }
    }
    ToolButton {
      id: o5
      ToolTip.text: "Object 5"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "pikachu.png"
        sourceSize.width: 48;
        sourceSize.height: 48;
      }
      onClicked: {
        CompObjectsPlugin.OnMode("ellipsoid")
      }
    }
  }
}