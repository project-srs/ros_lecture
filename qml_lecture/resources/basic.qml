import QtQuick 2.3
import QtQuick.Window 2.2
import QtQuick.Controls 1.2

Window {
    Column {
        Button {
            text: "Ok"
            onClicked: text_label.text = "ok"
        }
        Button {
            text: "Cancel"
            onClicked: text_label.text = "cancel"
        }
        Text {
            id: text_label
            text: "Hello World!"
        }
        Slider {
            onValueChanged : text_label.text = value
        }
    }
    visible: true
}