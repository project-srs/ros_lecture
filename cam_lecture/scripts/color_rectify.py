#!/usr/bin/env python
import numpy as np
import cv2
from jinja2 import Template
import webbrowser
import os

class PointList():
    def __init__(self, npoints):
        self.npoints = npoints
        self.ptlist = np.empty((npoints, 2), dtype=int)
        self.pos = 0

    def add(self, x, y):
        if self.pos < self.npoints:
            self.ptlist[self.pos, :] = [x, y]
            self.pos += 1
            return True
        return False

ref_color=[
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

def onMouse(event, x, y, flag, params):
    wname, img, ptlist = params
    if event == cv2.EVENT_MOUSEMOVE:
        img2 = np.copy(img)
        h, w = img2.shape[0], img2.shape[1]
        cv2.line(img2, (x, 0), (x, h - 1), (255, 0, 0))
        cv2.line(img2, (0, y), (w - 1, y), (255, 0, 0))
        cv2.imshow(wname, img2)

    if event == cv2.EVENT_LBUTTONDOWN:
        if ptlist.add(x, y):
            print('[%d] ( %d, %d )' % (ptlist.pos - 1, x, y))
            cv2.circle(img, (x, y), 3, (0, 0, 255), 3)
            cv2.imshow(wname, img)
        else:
            print('All points have selected.  Press ESC-key.')
        if(ptlist.pos == ptlist.npoints):
            print(ptlist.ptlist)
            pr = ptlist.ptlist[0]
            pb = ptlist.ptlist[1] - ptlist.ptlist[0]
            pc = ptlist.ptlist[2] - ptlist.ptlist[0]
            pa = ptlist.ptlist[3] - ptlist.ptlist[0]

            result = []
            for i in range(3 + 1):
                for j in range(5 + 1):
                    index = i * 6 + j
                    ry = i / 3.0
                    rx = j / 5.0
                    x = int(pr[0] + rx * (1 - ry) * pa[0] + (1 - rx) * ry * pb[0] + rx * ry * pc[0])
                    y = int(pr[1] + rx * (1 - ry) * pa[1] + (1 - rx) * ry * pb[1] + rx * ry * pc[1])
                    # print(rx,ry,"->",x,y)
                    cv2.circle(img, (x, y), 3, (100, 100, 255), 3)

                    ref = ref_color[index]
                    cur = img[y][x]
                    # print("", ref[0], ref[1], ref[2], ref[3], cur[2], cur[1], cur[0], "")
                    result.append([ref[0], ref[1], ref[2], ref[3], cur[2], cur[1], cur[0]])


            print(result)
            r_gain = np.zeros(24)
            g_gain = np.zeros(24)
            b_gain = np.zeros(24)
            for index in range(24):
                ref = result[index][1:4]
                msr = result[index][4:7]
                r_gain[index] = float(msr[0]) / float(ref[0])
                g_gain[index] = float(msr[1]) / float(ref[1])
                b_gain[index] = float(msr[2]) / float(ref[2])
                print('#%02x%02x%02x' % (ref[0],ref[1],ref[2]))
            generateHtml(result)
            print(os.getcwd())
            uri = 'file://' + os.getcwd() + '/result.html'
            # uri = 'file:///' + 'home/ubuntu/result.html'
            webbrowser.open(uri)
            cv2.destroyAllWindows()

def generateHtml(results):
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
        <th>red gain</th>
        <th>{{ red_gain }}</th>
    </tr>
    <tr>
        <th>green gain</th>
        <th>{{ green_gain }}</th>
    </tr>
    <tr>
        <th>blue gain</th>
        <th>{{ blue_gain }}</th>
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

</body>
</html>
'''

    r_gain = np.zeros(24)
    g_gain = np.zeros(24)
    b_gain = np.zeros(24)
    for index in range(24):
        ref = results[index][1:4]
        msr = results[index][4:7]
        r_gain[index] = float(msr[0]) / float(ref[0])
        g_gain[index] = float(msr[1]) / float(ref[1])
        b_gain[index] = float(msr[2]) / float(ref[2])
        print('#%02x%02x%02x' % (ref[0],ref[1],ref[2]))

    gains = (np.mean(np.sort(r_gain)[6:18]), np.mean(np.sort(g_gain)[6:18]), np.mean(np.sort(b_gain)[6:18]))

    template = Template(html)
    data = {
    'red_gain' : gains[0],
    'green_gain' : gains[1],
    'blue_gain' : gains[2],
    'results' : [
        # {'name':'red', },
        # {'name':'red'}
    ]
    }

    for index in range(24):
        referrence = '#%02x%02x%02x' % (results[index][1],results[index][2],results[index][3])
        rect_r = min(max(results[index][4] / gains[0], 0), 255)
        rect_g = min(max(results[index][5] / gains[1], 0), 255)
        rect_b = min(max(results[index][6] / gains[2], 0), 255)
        rectify = '#%02x%02x%02x' % (rect_r, rect_g, rect_b)
        measure = '#%02x%02x%02x' % (results[index][4],results[index][5],results[index][6])
        data['results'].append({'name':results[index][0], 'ref': referrence, 'rect': rectify, 'msr': measure})

    print (template.render(data))
    with open('result.html', 'w') as f:
        f.write(template.render(data))

if __name__ == '__main__':
    img = cv2.imread("image_red.png")
    wname = "MouseEvent"
    cv2.namedWindow(wname)
    npoints = 4
    ptlist = PointList(npoints)
    cv2.setMouseCallback(wname, onMouse, [wname, img, ptlist])
    cv2.imshow(wname, img)
    cv2.waitKey()
    cv2.destroyAllWindows()