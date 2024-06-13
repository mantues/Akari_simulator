#!/usr/bin/env python
# coding:utf-8

import time

import rclpy
#from akari_client.color import Color, Colors
from akari_client.position import Positions
from akari_msgs.srv import (
    SetAllout,
    SetDisplayColor,
    SetDisplayColorRgb,
    SetDisplayImage,
    SetDisplayText,
    SetDout,
    SetPwmout,
    Trigger,
)
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image as sensorImage
from cv_bridge import CvBridge, CvBridgeError
import os
from ament_index_python.packages import get_package_share_directory
from PIL import ImageFont, ImageDraw, Image
import numpy as np
import enum

color_list ={'BLACK': (255,0,0),'NAVY': (0, 0, 127), 'DARKGREEN': (0, 127, 0), 'DARKCYAN': (0, 127, 127), 'MAROON': (127, 0, 0), 'PURPLE': (127, 0, 127),
'OLIVE': (127, 127, 0), 'LIGHTGREY': (192, 192, 192), 'DARKGREY': (127, 127, 127), 'BLUE': (0, 0, 255), 'GREEN': (0, 255, 0),
'CYAN': (0, 255, 0), 'RED': (255, 0, 0), 'MAGENTA': (255, 0, 255), 'YELLOW': (255, 255, 0), 'WHITE': (255, 255, 255), 'ORANGE': (255, 165, 0), 'GREENYELLOW': (173, 255, 47), 'PINK': (255, 0, 255)
}

# server
class M5Server(Node):
    def __init__(self):
        super().__init__("m5_server_node")
        # create service display color from name
        self._set_display_color_srv = self.create_service(
            SetDisplayColor, "set_display_color", self.set_display_color
        )
        # create service display color from rgb
        self._set_display_color_rgb_srv = self.create_service(
            SetDisplayColorRgb, "set_display_color_rgb", self.set_display_color_rgb
        )
        # create service display text
        self._set_display_text_srv = self.create_service(
            SetDisplayText, "set_display_text", self.set_display_text
        )
        # create service display image
        self._set_display_image_srv = self.create_service(
            SetDisplayImage, "set_display_image", self.set_display_image
        )
        # create service reset m5stack
        self._reset_m5_srv = self.create_service(Trigger, "reset_m5", self.reset_m5)
        
        self.bridge = CvBridge()
        self.package_dir = get_package_share_directory("akari_simulator")
        self.akari_image = cv2.imread(self.package_dir+'/image/akari.png')
        self.ipa_font = self.package_dir+'/font/ipaexg.ttf'
        self.font = ImageFont.truetype(self.ipa_font, 20)        
        # create publisher
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.akari_callback)
        self.m5_image = self.create_publisher(sensorImage, "/m5_image", 10)
        
        
    # SERVER CALL BACK
    def akari_callback(self) -> None:
        cv_msg = self.bridge.cv2_to_imgmsg(self.akari_image, "bgr8")
        #self.get_logger().info(f"Shape: {self.akari_image.shape}") #(200, 200, 3)
        self.m5_image.publish(cv_msg)
        
    def set_display_color(self, request, response):
        req_color = request.color.upper()
        self.get_logger().info(f"req_color: {req_color}")
        response.result = True
        if req_color in color_pair:
            try:
                #self.m5.set_display_color(Colors[req_color])
                color = (255, 255 ,255)
                self.akari_image = cv2.rectangle(self.akari_image, (0,0), (199, 199), color, thickness=-1)
            except BaseException as e:
                self.get_logger().error(e)
                response.result = False
        else:
            self.get_logger().warn(f"Color {req_color} can't display")
            response.result = False
        return response

    def set_display_color_rgb(self, request, response):
        r_color = request.r
        g_color = request.g
        b_color = request.b
        color = (r_color, g_color ,b_color)
        response.result = True
        try:
            self.akari_image = cv2.rectangle(self.akari_image, (0,0), (199, 199), color, thickness=-1)
        except BaseException as e:
            self.get_logger().error(e)
            response.result = False
        return response

    def set_display_text(self, request, response):
        req_text = request.text
        req_pos_x = request.pos_x
        req_pos_y = request.pos_y
        req_size = request.size
        req_text_color = request.text_color
        req_back_color = request.back_color
        req_refresh = request.refresh
        if req_text_color == req_back_color:
            text_color = (0, 0, 0)
            back_color = (255, 255, 255)
        else:
            if req_text_color in color_list.keys():
                text_color = color_list[req_text_color]
            else:
                text_color = (0,0,0)
            print(text_color)
            if req_back_color in color_list.keys():
                back_color = color_list[req_back_color]
            else:
                back_color = (255,255,255)
        self.get_logger().info(f"Text color: {text_color}")
        if req_refresh == True:
            self.akari_image = cv2.rectangle(self.akari_image, (0,0), (199, 199), color=(255, 255, 255), thickness=-1)
            time.sleep(0.05)
            
        response.result = True
        self.get_logger().info(f"Text position: {req_pos_x, req_pos_y}")
        try:
            #self.get_logger().info(f"image shape: {self.akari_image.shape}")
            PIL_image = Image.fromarray(self.akari_image)
            draw_image = ImageDraw.Draw(PIL_image)
            text_size = draw_image.multiline_textsize(req_text, font=self.font)
            self.get_logger().info(f"Text size: {text_size}")
            draw_image.rectangle(
                  [(req_pos_x, req_pos_y), (req_pos_x+text_size[0], req_pos_y+text_size[1])], fill=back_color, outline=back_color, width=1
                  )
            draw_image.text((req_pos_x, req_pos_y), req_text, text_color, font=self.font)
            cvt_image = np.array(PIL_image)
            #cvt_image = cv2.cvtColor(cvt_image, cv2.COLOR_BGR2RGB)
            #self.akari_image = cv2.cvtColor(cvt_image, cv2.COLOR_RGBA2BGRA)
            self.akari_image = cvt_image
            time.sleep(2)
        except BaseException as e:
            self.get_logger().error(e)
            response.result = False
        return response

    def set_display_image(self, request, response):
        filepath = request.filepath
        pos_x = request.pos_x
        pos_y = request.pos_y
        scale = request.scale
        response.result = True
        try:
            #self.m5.set_display_image(filepath, pos_x, pos_y, scale)
            self.akari_image = cv2.imread(filepath)
            cv_msg = self.bridge.cv2_to_imgmsg(self.akari_image, "bgr8")
            self.m5_image.publish(cv_msg)
        except BaseException as e:
            self.get_logger().error(e)
            response.result = False
        return response

    def reset_m5(self, request, response):
        response.result = True
        try:
            self.get_logger().info(f"RESET IMAGE")
            self.akari_image = cv2.rectangle(self.akari_image,
                (10, 100),
                (175, 120),
                color = self.colors.WHITE,
                thickness = -1,
                lineType = cv2.LINE_4)
            self.akari_image = cv2.putText(self.akari_image,
                text = 'RESET IMAGE',
                org = (10, 120),
                fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                fontScale = 0.8,
                color = self.colors.YELLOW,
                thickness = 2,
                lineType = cv2.LINE_4)
            cv_msg = self.bridge.cv2_to_imgmsg(self.akari_image, "bgr8")
            self.m5_image.publish(cv_msg)
            time.sleep(4)
            self.akari_image = cv2.imread(self.package_dir+'/image/akari.png')
        except BaseException as e:
            self.get_logger().error(e)
            response.result = False
        return response

def main(args = None):
    rclpy.init(args = args)
    server = M5Server()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
