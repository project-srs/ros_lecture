import QtQuick 2.3
import QtQuick.Window 2.2
import QtQuick.Controls 1.2

Window {
    ScrollView {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        anchors.topMargin: 10
        anchors.bottomMargin: 10

        Column {
            Button {
                text: "Ok"
                onClicked: mediator.newString("aa")
            }
            Button {
                text: "Cancel"
                onClicked: model.revert()
            }
            Text {
                text: "Hello World!"
                font.family: "Helvetica"
                font.pointSize: 12
                color: "red"
                id: text1
            }
            Slider {
                value          : storage.get('Setting 1') || 0
                onValueChanged : _text.text = value
            }
            ListView {
                anchors.fill: parent

                model: mediator.strings
                delegate: Text {
                    anchors.left: parent.left
                    anchors.right: parent.right
                    height: 25

                    text: modelData
                }
            }
        }

    
    }

    visible: true
    width: 360
    height: 360
}