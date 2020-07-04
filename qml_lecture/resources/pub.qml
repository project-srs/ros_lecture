import QtQuick 2.3
import QtQuick.Window 2.2
import QtQuick.Controls 1.2

Window {
    Column {
        Button {
            text: "OK"
            onClicked: ros_string.publishString("OK")
        }
        Button {
            text: "Cancel"
            onClicked: ros_string.publishString("Cancel")
        }
    }
    visible: true
}