# see https://blog.alexandruioan.me/2017/01/31/the-2017-university-of-bristol-arm-hackathon for more details
import math
from http.server import BaseHTTPRequestHandler, HTTPServer, urllib
from sys import argv
import sys
import serial
import threading
import queue
import numpy as np
import time

q = queue.Queue()

radius = 250

DOWN = b'c130'
UP = b'c70'

INIT_A = b'a128'
INIT_B = b'b84'

MOVE = b'm'

# serial flag for debugging
SEND = True

if SEND:
    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.write(UP)
    ser.write(INIT_A)
    ser.write(INIT_B)
    ser.write(MOVE)

# separate thread for sending data to serial
def serial_send(q):
    while True:
        to_send = q.get()

        if SEND:
            print("Write start")
            ser.write(to_send)
            print("Write end")
            #ser.flush()
        q.task_done()

thrSend = threading.Thread(target=serial_send, args=(q,))
thrSend.setDaemon(True)
thrSend.start()

def serial_read():
    while True:
        print(ser.readline())

thrRecv = threading.Thread(target=serial_read)
thrRecv.setDaemon(True)
thrRecv.start()

class req_handler(BaseHTTPRequestHandler):

    prevX = -1
    prevY = -1

    # this runs when points are POSTed to the server
    # it triggers the calculation of the angles of the servos
    # and puts them in a queue
    # the tread at the other end of the queue sends the data over serial
    def do_POST(self):
        length = int(self.headers['Content-Length'])
        post_data = urllib.parse.parse_qs(self.rfile.read(length).decode('utf-8'))

        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

        count = int((post_data['count'])[0])
        q.put_nowait(UP)

        for i in range(count):
            pointX = float((post_data['p' + str(i) + 'x'])[0])
            pointY = float((post_data['p' + str(i) + 'y'])[0])

            if pointX == 0 and pointY == -1: # pen down
                q.put_nowait(DOWN)
                print("pen down")
                continue
            elif pointX == -1 and pointY == 0: # pen up
                q.put_nowait(UP)
                print("pen up")
                continue

            # don't draw points that are too close
            if (req_handler.prevX, req_handler.prevY) != (-1, -1):
                if math.sqrt((req_handler.prevX - pointX)**2 + (req_handler.prevY - pointY)**2) < 0.5:
                    continue

            # timing
            #t0 = time.time()
            (theta1, theta2) = calculateANgle(pointX, pointY)
            #t1 = time.time()
            #total = t1 - t0
            #print("Time: ", total)

            (a, b) = (int(round(theta1)), int(round(theta2)))

            to_send = ('a' + str(a) + 'b' + str(b) + 'm' + '\n').encode('ascii') # + '\n'
            q.put_nowait(to_send)

            # save the current position
            (req_handler.prevX, req_handler.prevY) = (pointX, pointY)

        # TODO return to center?
        q.put_nowait(UP)

# https://uk.mathworks.com/help/fuzzy/examples/modeling-inverse-kinematics-in-a-robotic-arm.html
# pregenerate grid
theta1range = np.arange(0, math.pi, 0.01)
theta2range = np.arange(0, math.pi, 0.01)

THETA1, THETA2 = np.meshgrid(theta1range, theta2range)

X_pred = radius * np.cos(THETA1) + radius * np.cos(THETA1 + THETA2)
Y_pred = radius * np.sin(THETA1) + radius * np.sin(THETA1 + THETA2)

grid = np.dstack((X_pred, Y_pred))

def calculateAngle(pointX, pointY):
    min_dist = 100000
    min_theta1 = 0
    min_theta2 = 0

    # slow solution
    # ~0.12s
    #for theta1 in np.arange(0, math.pi, 0.01):
    #    for theta2 in np.arange(0, math.pi, 0.01):
    #        x_pred = radius * math.cos(theta1) + radius * math.cos(theta1 + theta2)
    #        y_pred = radius * math.sin(theta1) + radius * math.sin(theta1 + theta2)

    #        look_dist = math.sqrt((x_pred - pointX) ** 2 + (y_pred - pointY) ** 2)
    #        if look_dist < min_dist:
    #            min_dist = look_dist
    #            min_theta1 = theta1
    #            min_theta2 = theta2

    # numpy solution
    # ~0.005s
    # generate 3D array of repeated target point
    point = np.array([[[pointX, pointY]]])
    point3D = np.repeat(np.repeat(point, X_pred.shape[0], axis = 0), X_pred.shape[1], axis = 1)
    # create 3D array with potential X and Y values
    grid = np.dstack((X_pred, Y_pred))
    # compute the Euclidean distance
    diff = np.subtract(point3D, grid)
    dists = np.linalg.norm(diff, ord = 2, axis = 2)
    # find the minimum distance (grid point closest to the target point)
    idx1, idx2 = np.unravel_index(dists.argmin(), dists.shape)
    # extract its theta values
    min_theta1 = THETA1[idx1][idx2]
    min_theta2 = THETA2[idx1][idx2]

    return (math.degrees(min_theta1), math.degrees(min_theta2))

# start the HTTP server, binding it to all addresses, port 1180
def run(server_class = HTTPServer, handler_class = req_handler, port = 1180):
    server_address = ('0.0.0.0', port)
    httpd = server_class(server_address, handler_class)
    print('Starting httpd...')
    httpd.serve_forever()

def main():
    try:
        if len(argv) == 2:
            run(port = int(argv[1]))
        else:
            run()
    except KeyboardInterrupt:
        print("Exiting...")
        sys.exit()

if __name__ == "__main__":
    main()
