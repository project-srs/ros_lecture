#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from jinja2 import Template
import webbrowser
import os

ref_colors=[
    ["DarkSkin", 115, 82, 69],
    ["LightSkin", 204, 161, 141],
    ["BlueSky", 101, 134, 179],
    ["Foliage", 89, 109, 61],
    ["BlueFlower", 141, 137, 194],
    ["BluishGreen", 132, 228, 208],

    ["Orange", 249, 118, 35],
    ["PurplishBlue", 80, 91, 182],
    ["ModerateRed", 222, 91, 125],
    ["Purple", 91, 63, 123],
    ["YellowGreen", 173, 232, 91],
    ["OrangeTellow", 255, 164, 26],

    ["Blue", 44, 56, 142],
    ["Green", 74, 148, 81],
    ["Red", 179, 42, 50],
    ["Yellow", 250, 226, 21],
    ["Magenta", 191, 81, 160],
    ["Cyan", 6, 142, 172],

    ["White", 252, 252, 252],
    ["Neutral0.02", 230, 230, 230],
    ["Neutral0.42", 200, 200, 200],
    ["Neutral0.68", 143, 143, 142],
    ["Neutral1.00", 100, 100, 100],
    ["Black", 50, 50, 50]
]

html = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <title>My Webpage</title>
</head>
<body>
<h1>Result</h1>
<table border="1">
    <tr>
        <th>color</th>
        <th>result gian(Median 12 points)</th>
        <th>average(all)</th>
        <th>std(all)</th>
    </tr>
    <tr>
        <th>red gain</th>
        <th>{{ gains[0] }}</th>
        <th>{{ avrs[0] }}</th>
        <th>{{ stds[0] }}</th>
    </tr>
    <tr>
        <th>green gain</th>
        <th>{{ gains[1] }}</th>
        <th>{{ avrs[1] }}</th>
        <th>{{ stds[1] }}</th>
    </tr>
    <tr>
        <th>blue gain</th>
        <th>{{ gains[2] }}</th>
        <th>{{ avrs[2] }}</th>
        <th>{{ stds[2] }}</th>
    </tr>
</table>


<h1>Detail</h1>
<table border="1">
    <tr>
        <th>color name</th>
        <th>refrence color</th>
        <th>rectify color</th>
        <th>measure color</th>
    </tr>

    {% for item in results %}
    <td >{{ item.name }}</td>
        <td bgcolor="{{ item.ref }}">{{ item.ref }}</td>
        <td bgcolor="{{ item.rect }}">{{ item.rect }}</td>
        <td bgcolor="{{ item.msr }}">{{ item.msr }}</td>
    </tr>
    {% endfor %}
</table>

<h1>image</h1>
<img src="measure_image.jpg"/>

</body>
</html>
'''

class ColorRectify:
    def __init__(self):
        rospy.init_node("ColorRectify")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_raw", Image, self.image_callback, queue_size=1)
        self.window_name = "rectify"
        self.stage = 0
        self.ptlist = np.empty((4, 2), dtype=int)
        self.last_image = None
        self.last_mouse = np.zeros(2, dtype=int)
        self.last_show_stamp = rospy.get_rostime()
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        print("click to capture")
            
    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        if self.stage == 0:
            self.last_image = np.array(frame, dtype=np.uint8)
        self.display()

    def mouse_callback(self, event, x, y, flag, params):
        if event == cv2.EVENT_MOUSEMOVE:
            self.last_mouse = np.array([x, y])
        elif event == cv2.EVENT_LBUTTONDOWN:
            if self.stage < 5:
                index = self.stage-1
                self.add_point(index, x, y)

            self.stage += 1

            if self.stage == 1:
                print("click on the four corners LT -> LB -> RB ->RT")
            elif self.stage == 5:
                print("click to analize")
            elif self.stage == 6:
                cv2.imwrite("measure_image.jpg", self.last_image)
                measure_colors, gains, avrs, stds = self.generateResult(self.last_image, self.ptlist, ref_colors)
                uri = self.generateHtml(ref_colors, measure_colors, gains, avrs, stds)
                webbrowser.open(uri)
                rospy.signal_shutdown("finish")

        self.display()

    def display(self):
        if self.last_image is None:
            return

        now = rospy.get_rostime()
        diff = (now - self.last_show_stamp).to_sec()
        if diff < 0.1:
            return
        self.last_show_stamp = now

        image = self.last_image.copy()
        x, y = self.last_mouse[0], self.last_mouse[1]
        h, w = image.shape[0], image.shape[1]

        if 1 <= self.stage and self.stage < 5:
            cv2.line(image, (x, 0), (x, h - 1), (255, 0, 0))
            cv2.line(image, (0, y), (w - 1, y), (255, 0, 0))
        
        if self.stage == 5:
            result =self. get_color_points(self.ptlist)
            for r in result:
                cv2.circle(image, (r[0], r[1]), 3, (100, 100, 255), 3)

        point_num = min(max(self.stage-1, 0), 4)
        for i in range(point_num):
            cv2.circle(image, (self.ptlist[i,0], self.ptlist[i,1]), 3, (0, 0, 255), 3)
        cv2.imshow(self.window_name, image)   
        cv2.waitKey(1)

    def add_point(self, i, x, y):
        if i < 4:
            self.ptlist[i, :] = [x, y]
            return True
        return False

    def get_color_points(self, ptlist):
        pr = ptlist[0]
        pb = ptlist[1] - ptlist[0]
        pc = ptlist[2] - ptlist[0]
        pa = ptlist[3] - ptlist[0]

        result = []
        for i in range(3 + 1):
            for j in range(5 + 1):
                index = i * 6 + j
                ry = i / 3.0
                rx = j / 5.0
                x = int(pr[0] + rx * (1 - ry) * pa[0] + (1 - rx) * ry * pb[0] + rx * ry * pc[0])
                y = int(pr[1] + rx * (1 - ry) * pa[1] + (1 - rx) * ry * pb[1] + rx * ry * pc[1])
                result.append([x, y])
        return result
    
    def generateResult(self, image, ptlist, ref_colors):
        measure_points = self. get_color_points(self.ptlist)
        measure_colors = []
        for m in measure_points:
            color = image[m[1]][m[0]]
            measure_colors.append([color[2], color[1], color[0]])

        assert len(ref_colors) == 24
        assert len(measure_colors) == 24
        r_gain = np.zeros(24)
        g_gain = np.zeros(24)
        b_gain = np.zeros(24)
        for index in range(24):
            r_gain[index] = float(measure_colors[index][0]) / float(ref_colors[index][1])
            g_gain[index] = float(measure_colors[index][1]) / float(ref_colors[index][2])
            b_gain[index] = float(measure_colors[index][2]) / float(ref_colors[index][3])
        
        avrs = [np.average(r_gain), np.average(g_gain), np.average(b_gain)]
        stds = [np.std(r_gain), np.std(g_gain), np.std(b_gain)]
        gains = [np.mean(np.sort(r_gain)[6:18]), np.mean(np.sort(g_gain)[6:18]), np.mean(np.sort(b_gain)[6:18])]
        return measure_colors, gains, avrs, stds

    def generateHtml(self, ref_colors, measure_colors, gains, avrs, stds):
        template = Template(html)
        data = {
            "gains": gains,
            "avrs": avrs,
            "stds": stds,
            'results': [ ]
        }

        for index in range(24):
            rect_r = min(max(measure_colors[index][0] / gains[0], 0), 255)
            rect_g = min(max(measure_colors[index][1] / gains[1], 0), 255)
            rect_b = min(max(measure_colors[index][2] / gains[2], 0), 255)
            referrence = '#%02x%02x%02x' % (ref_colors[index][1],ref_colors[index][2],ref_colors[index][3])
            rectify = '#%02x%02x%02x' % (rect_r, rect_g, rect_b)
            measure = '#%02x%02x%02x' % (measure_colors[index][0],measure_colors[index][1],measure_colors[index][2])
            data['results'].append({'name':ref_colors[index][0], 'ref': referrence, 'rect': rectify, 'msr': measure})

        with open('result.html', 'w') as f:
            f.write(template.render(data))

        return 'file://' + os.getcwd() + '/result.html'

if __name__ == '__main__':
    ColorRectify()
    rospy.spin()