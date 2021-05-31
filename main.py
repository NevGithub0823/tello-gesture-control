#!/usr/bin/env python
# -*- coding: utf-8 -*-
import configargparse

import cv2 as cv

from gestures.tello_gesture_controller import TelloGestureController
from utils import CvFpsCalc

from djitellopy import Tello
from gestures import *

import threading


def get_args():
    print('## Reading configuration ##')
    parser = configargparse.ArgParser(default_config_files=['config.txt'])

    parser.add('-c',
               '--my_config',
               required=False,
               is_config_file=True,
               help='config file path')
    parser.add("--device", type=int)
    parser.add("--width", help='cap width', type=int)
    parser.add("--height", help='cap height', type=int)
    parser.add("--is_keyboard",
               help='To use Keyboard control by default',
               action='store_true')
    parser.add('--use_static_image_mode',
               help='True if running on photos',
               action='store_true')
    parser.add("--min_detection_confidence",
               help='min_detection_confidence',
               type=float)
    parser.add("--min_tracking_confidence",
               help='min_tracking_confidence',
               type=float)
    parser.add("--buffer_len", help='Length of gesture buffer', type=int)
    parser.add(
        "--use_webcam",
        help='Use webcam instead of tello camera for detecting gestures',
        action='store_true')

    args = parser.parse_args()

    return args


def select_mode(key, mode):
    number = -1
    if 48 <= key <= 57:  # 0 ~ 9
        number = key - 48
    if key == 110:  # n
        mode = 0
    if key == 107:  # k
        mode = 1
    if key == 104:  # h
        mode = 2
    return number, mode


def main():
    # init global vars
    global gesture_buffer
    global gesture_id
    global battery_status

    # Argument parsing
    args = get_args()
    USE_WEBCAM = args.use_webcam
    KEYBOARD_CONTROL = args.is_keyboard
    WRITE_CONTROL = False
    in_flight = False

    # Camera preparation
    tello = Tello()
    if not tello.connect() or not tello.streamon():
        exit()

    cap = tello.get_frame_read()
    cap_webcam = cv.VideoCapture(0)

    # Init Tello Controllers
    gesture_controller = TelloGestureController(tello)
    keyboard_controller = TelloKeyboardController(tello)

    gesture_detector = GestureRecognition(args.use_static_image_mode,
                                          args.min_detection_confidence,
                                          args.min_tracking_confidence)
    gesture_buffer = GestureBuffer(buffer_len=args.buffer_len)

    def tello_control(key, keyboard_controller, gesture_controller):
        global gesture_buffer

        if KEYBOARD_CONTROL:
            keyboard_controller.control(key)
        else:
            gesture_controller.gesture_control(gesture_buffer)

    def tello_battery(tello):
        global battery_status
        battery_status = tello.get_battery()

    # FPS Measurement
    cv_fps_calc = CvFpsCalc(buffer_len=10)

    mode = 0
    number = -1
    battery_status = -1

    # tello.move_down(20) # idk what this line was for, commenting it out

    while True:
        fps = cv_fps_calc.get()

        # Process Key (ESC: end)
        key = cv.waitKey(1) & 0xff
        if key == 27:  # ESC
            tello.land()
            tello.end()
            cv.destroyAllWindows()
            break
        elif key == 32:  # Space

            # sometimes it fails, doesn't matter, keep going
            try:
                if not in_flight:
                    # Take-off drone
                    tello.takeoff()
                    in_flight = True

                elif in_flight:
                    # Land tello
                    tello.land()
                    in_flight = False
            except:
                pass

        elif key == ord('k'):
            mode = 0
            KEYBOARD_CONTROL = True
            WRITE_CONTROL = False
            tello.send_rc_control(0, 0, 0, 0)  # Stop moving
        elif key == ord('g'):
            KEYBOARD_CONTROL = False
        elif key == ord('n'):
            mode = 1
            WRITE_CONTROL = True
            KEYBOARD_CONTROL = True

        if WRITE_CONTROL:
            number = -1
            if 48 <= key <= 57:  # 0 ~ 9
                number = key - 48

        # Camera capture
        # s is not used, but is returned
        s, image = cap_webcam.read() if USE_WEBCAM else 0, cap.frame

        debug_image, gesture_id = gesture_detector.recognize(
            image, number, mode)
        gesture_buffer.add_gesture(gesture_id)

        # Start control thread
        threading.Thread(target=tello_control,
                         args=(
                             key,
                             keyboard_controller,
                             gesture_controller,
                         )).start()
        threading.Thread(target=tello_battery, args=(tello, )).start()

        debug_image = gesture_detector.draw_info(debug_image, fps, mode,
                                                 number)

        # Battery status
        battery_str_pos = (5, 100) if USE_WEBCAM else (5, 720 - 5)
        cv.putText(debug_image, "Battery: {}".format(battery_status),
                   battery_str_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Control status
        control_str_pos = (5, 65)
        cv.putText(
            debug_image,
            '{} control'.format('Keyboard' if KEYBOARD_CONTROL else 'Gesture'),
            control_str_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Other
        cv.putText(debug_image, 'Webcam: {}'.format(USE_WEBCAM.__str__()), (5, 135),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (255, 186, 82), 2)
        cv.putText(debug_image, 'In flight: {}'.format(in_flight.__str__()),
                   (5, 170), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 186, 82), 2)
        cv.putText(debug_image, 'Can takeoff: {}'.format((battery_status > 11).__str__()),
                   (5, 205), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 120, 8), 2)

        # Image rendering
        cv.imshow('Tello Gesture Recognition', debug_image)


if __name__ == '__main__':
    main()
