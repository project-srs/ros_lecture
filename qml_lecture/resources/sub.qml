import QtQuick 2.3
import QtQuick.Window 2.2
import QtQuick.Controls 1.2

Window {
    Column {
        Text {
            id: label_text
            text: "unknown"
        }
    }
    Connections{
        target:ros_string
        onSubscribeString: label_text.text = text
    }
    visible: true
}