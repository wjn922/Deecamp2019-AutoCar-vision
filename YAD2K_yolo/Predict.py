#!/usr/bin/env python

from __future__ import division
import glob
import numpy as np

import rospy
import os, time
#os.environ["CUDA_DEVICE_ORDER"] = 'PCI_BUS_ID'
#os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
import PIL
from PIL import ImageDraw, ImageFont
import PIL.Image as PImage
import cv2
import tensorflow as tf
from keras import backend as K
from keras.layers import Input, Lambda, Conv2D
from keras.models import load_model, Model
from yad2k.models.keras_yolo import (yolo_body, yolo_eval, yolo_head, yolo_loss)
import change_color
from discal import cal_dis
from lane_detection.lane_detection import detect_single
from Pannal_Crossing.Image_Manipulation import resize_img
from Pannal_Crossing.Zebra_Crossing_Detection import zebra_crossing_detection

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Config:
    abs_path = "/home/pro/Documents/CAR/src/YAD2K/"
    model_path = abs_path+'trained_stage_3_best_0126.h5'
    test_path = abs_path+'dataset/original_mixed_adj_test'
    output_path = abs_path+'images/out'
    classes_path = abs_path+"classes.txt"
    score_threshold = 0.4
    iou_threshold = 0.3
    yolo_anchors = np.array(
        ((0.57273, 0.677385), (1.87446, 2.06253), (3.33843, 5.47434),
         (7.88282, 3.52778), (9.77052, 9.16828)))
    class_names = {0: "stop", 1: "obstacle", 2: "slow", 3: "car"}
    class_colors = [(255, 0, 0), (127, 0, 255), (0, 255, 255), (127, 255, 0)]
    raw_img_shape=(480, 640)
cfg = Config()


class Predictor:

    def __init__(self):      
        self.imgprocess_count = 0
        self.imgprocess_interval = 10

        self.boxes = None
        self.scores = None
        self.classes = None
        self.sess = None
        self.yolo_model = None
        self.model_image_size = None
        self.input_image_shape = None
        self.ped_sign = resize_img(cv2.imread("/home/pro/Documents/CAR/src/YAD2K/Pannal_Crossing.jpg"), width=60, height=60)
        self.predict_init()
        print "start"
        rospy.init_node('YAD2K', anonymous=True)
        rospy.Subscriber('Detection/image', Image, self.callback_image)

    def callback_image(self, img):
 
        IMG = CvBridge().imgmsg_to_cv2(img)
        #print(type(IMG))
        # st = time.time()
        # #img_name = os.path.basename(img_path)
        img_src = IMG  #cv2.imread(img_path)
        #print(img_src.shape)
        out_boxes, _, out_classes = self.predict_bbox(img_src)
        img_show_obj = predictor.draw_bboxes(img_src, out_boxes, out_classes)
        img_show_obj = predictor.zebra_crossing(img_src, img_show_obj)
        dists = predictor.get_dist(out_boxes, out_classes)
        img_show_obj = detect_obstacle(img_show_obj, out_boxes, dists/100., out_classes)
        #save_path_obj = "%s/%s_obj.jpg" % (save_root, img_name.replace(".jpg", ""))
        #img_show_obj = detect_single(img_src, img_show_obj, mtx, dist)
        #save_path_lane = "%s/%s_lane.jpg" % (save_root, img_name.replace(".jpg", ""))
        # t = (time.time() - st)*1000
        # print("[%d]--time: %.2f ms"%(img_cnt, t))
        #cv2.imwrite(save_path_obj, img_show_obj)


        # show a frame
        cv2.imshow("YAD2K", img_show_obj)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break
        
        #cv2.imshow('lane_img', IMG)
        cv2.waitKey(1)

    def get_classes(self, classes_path):
        '''loads the classes'''
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names

    def process_data(self, images, boxes=None):
        '''processes the data'''
        images = [PIL.Image.fromarray(i) for i in images]
        orig_size = np.array([images[0].width, images[0].height])
        orig_size = np.expand_dims(orig_size, axis=0)
        # Image preprocessing.
        processed_images = [i.resize((416, 416), PIL.Image.BICUBIC) for i in images]
        processed_images = [np.array(image, dtype=np.float) for image in processed_images]
        processed_images = [image/255. for image in processed_images]
        if boxes is not None:
            # Box preprocessing.
            # Original boxes stored as 1D list of class, x_min, y_min, x_max, y_max.
            boxes = [box.reshape((-1, 5)) for box in boxes]
            # Get extents as y_min, x_min, y_max, x_max, class for comparision with
            # model output.
            # Get box parameters as x_center, y_center, box_width, box_height, class.
            boxes_xy = [0.5 * (box[:, 3:5] + box[:, 1:3]) for box in boxes]
            boxes_wh = [box[:, 3:5] - box[:, 1:3] for box in boxes]
            boxes_xy = [boxxy / orig_size for boxxy in boxes_xy]
            boxes_wh = [boxwh / orig_size for boxwh in boxes_wh]
            boxes = [np.concatenate((boxes_xy[i], boxes_wh[i], box[:, 0:1]), axis=1) for i, box in enumerate(boxes)]
            # find the max number of boxes
            max_boxes = 0
            for boxz in boxes:
                if boxz.shape[0] > max_boxes:
                    max_boxes = boxz.shape[0]
            # add zero pad for training
            for i, boxz in enumerate(boxes):
                if boxz.shape[0]  < max_boxes:
                    zero_padding = np.zeros( (max_boxes-boxz.shape[0], 5), dtype=np.float32)
                    boxes[i] = np.vstack((boxz, zero_padding))
            return np.array(processed_images), np.array(boxes)
        else:
            return np.array(processed_images)

    def create_model(self, anchors, class_names, load_pretrained=False, freeze_body=True):
        '''
        returns the body of the model and the model

        # Params:

        load_pretrained: whether or not to load the pretrained model or initialize all weights

        freeze_body: whether or not to freeze all weights except for the last layer's

        # Returns:

        model_body: YOLOv2 with new output layer

        model: YOLOv2 with custom loss Lambda layer

        '''

        detectors_mask_shape = (13, 13, 5, 1)
        matching_boxes_shape = (13, 13, 5, 5)

        # Create model input layers.
        image_input = Input(shape=(416, 416, 3))
        boxes_input = Input(shape=(None, 5))
        detectors_mask_input = Input(shape=detectors_mask_shape)
        matching_boxes_input = Input(shape=matching_boxes_shape)

        # Create model body.
        yolo_model = yolo_body(image_input, len(anchors), len(class_names))
        topless_yolo = Model(yolo_model.input, yolo_model.layers[-2].output)

        if load_pretrained:
            # Save topless yolo:
            topless_yolo_path = os.path.join('model_data', 'yolo_topless.h5')
            if not os.path.exists(topless_yolo_path):
                print("CREATING TOPLESS WEIGHTS FILE")
                yolo_path = os.path.join('model_data', 'yolo.h5')
                model_body = load_model(yolo_path)
                model_body = Model(model_body.inputs, model_body.layers[-2].output)
                model_body.save_weights(topless_yolo_path)
            topless_yolo.load_weights(topless_yolo_path)

        if freeze_body:
            for layer in topless_yolo.layers:
                layer.trainable = False
        final_layer = Conv2D(len(anchors) * (5 + len(class_names)), (1, 1), activation='linear')(topless_yolo.output)

        model_body = Model(image_input, final_layer)

        # Place model loss on CPU to reduce GPU memory usage.
        with tf.device('/cpu:0'):
            # TODO: Replace Lambda with custom Keras layer for loss.
            model_loss = Lambda(
                yolo_loss,
                output_shape=(1,),
                name='yolo_loss',
                arguments={'anchors': anchors,
                           'num_classes': len(class_names)})([
                model_body.output, boxes_input,
                detectors_mask_input, matching_boxes_input
            ])

        model = Model(
            [model_body.input, boxes_input, detectors_mask_input,
             matching_boxes_input], model_loss)

        return model_body, model

    def predict_init(self):

        output_path = cfg.output_path
        if not os.path.exists(output_path):
            print('Creating output path {}'.format(output_path))
            os.makedirs(output_path)
        anchors = cfg.yolo_anchors
        classes_path = cfg.classes_path
        model_path = cfg.model_path
        class_names = self.get_classes(classes_path)
        self.yolo_model, _ = self.create_model(anchors, class_names)
        self.yolo_model.load_weights(model_path)
        self.sess = K.get_session()
        self.model_image_size = self.yolo_model.layers[0].input_shape[1:3]
        yolo_outputs = yolo_head(self.yolo_model.output, anchors, len(class_names))
        self.input_image_shape = K.placeholder(shape=(2, ))
        self.graph = tf.get_default_graph()
        self.boxes, self.scores, self.classes = yolo_eval(
            yolo_outputs,
            self.input_image_shape,
            score_threshold=cfg.score_threshold,
            iou_threshold=cfg.iou_threshold)

    '''img: BGR ndarray'''
    def image_preprocess(self, image):
        image = change_color.color_balance(image)
        image = change_color.change_light(image,L=1.3, S=1)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PImage.fromarray(image)
        #print(self.model_image_size)
        resized_image = image.resize(
            tuple(reversed(self.model_image_size)), PImage.BICUBIC)
        image_data = np.array(resized_image, dtype='float32')
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.
        return image_data

    def predict_bbox(self, image):
        image_data = self.image_preprocess(image)
        #print(image_data.shape)
        #print(image_data[0,0,0])
    
        with self.graph.as_default():
            out_boxes, out_scores, out_classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={
                    self.yolo_model.input: image_data,
                    self.input_image_shape: [image.shape[0], image.shape[1]],
                    K.learning_phase(): 0})
                
        return out_boxes, out_scores, out_classes
 
    def draw_bboxes(self, image, out_boxes, out_classes, out_scores=None):
        image = cv2pil(image)
        font = ImageFont.truetype(
            font=cfg.abs_path+'font/FiraMono-Medium.otf',
            size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
        thickness = (image.size[0] + image.size[1]) // 300
        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = cfg.class_names[c]
            box = out_boxes[i]
            dist = int(cal_dis(box[0], box[2], predicted_class)/10)
            if dist == -1:
                dist = ""
            else:
                dist = "%d" % (dist)
            label = '%s %s cm' % (predicted_class, dist)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, font)
            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])
            for i in range(thickness):
                draw.rectangle(
                    [left + i, top + i, right - i, bottom - i],
                    outline=cfg.class_colors[c])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=cfg.class_colors[c])
            draw.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw
        return pil2cv(image)

    def get_dist(self, boxes, classes):
        n = len(boxes)
        dists = np.zeros(n)
        for i in range(n):
            dists[i] = cal_dis(boxes[i, 0], boxes[i, 2], classes[i])
        return dists

    def zebra_crossing(self, img_src, img_view):
        detected, dist = zebra_crossing_detection(img_src)
        # Draw the lane onto the warped blank image
        if detected:
            img_view[10:10+self.ped_sign.shape[0],550:550+self.ped_sign.shape[1],:]=self.ped_sign[:,:,:]
            img_view[10+self.ped_sign.shape[0]:35+self.ped_sign.shape[0],550:550+self.ped_sign.shape[1],:]=255
            font = cv2.FONT_HERSHEY_SIMPLEX
            img_view = cv2.putText(img_view, '%.2fm' %dist , (550,30+self.ped_sign.shape[0]), font, 0.6, (0, 0, 0), 2)
        return img_view
        # cv2.imwrite('Data_Set\Encounter2\detected_%s'%filebox[i], img_view)

    def adj_boxes(self, out_boxes):
        rate_h = cfg.raw_img_shape[0] / self.model_image_size[0]
        rate_w = cfg.raw_img_shape[1] / self.model_image_size[1]
        out_boxes[:, 0] *= rate_w
        out_boxes[:, 1] *= rate_h
        out_boxes[:, 2] *= rate_w
        out_boxes[:, 3] *= rate_h
        out_boxes.astype(np.int)
        out_boxes[:, 0:2] = np.maximum(out_boxes[:,0:2], 0)
        out_boxes[:, 2] = np.minimum(out_boxes[:, 2], cfg.raw_img_shape[1] - 1)
        out_boxes[:, 3] = np.minimum(out_boxes[:, 3], cfg.raw_img_shape[0] - 1)
        return out_boxes

def cv2pil(cvimg):
    image = cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB)
    image = PImage.fromarray(image)
    return image

def pil2cv(pilimg):
    image = np.array(pilimg)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image

def main():
    print("loading model...")
    predictor = Predictor()
    img_root = "./dataset/original_mixed_adj_test"
    save_root = "./images/%s_show" % os.path.basename(img_root)
    if not os.path.exists(save_root):
        os.makedirs(save_root)
    mtx = np.load("lane_detection/mtx.npy")
    dist = np.load("lane_detection/dist.npy")
    print("start detection...")
    total_st = time.time()
    img_paths = glob.glob(img_root + '/*.jpg')
    for i, img_path in enumerate(img_paths):
        st = time.time() 
        img_name = os.path.basename(img_path)
        img_src = cv2.imread(img_path)
        out_boxes, _, out_classes = predictor.predict_bbox(img_src)
        img_show_obj = predictor.draw_bboxes(img_src, out_boxes, out_classes)
        img_show_obj = predictor.zebra_crossing(img_src, img_show_obj)
        dists = predictor.get_dist(out_boxes, out_classes)
        img_show_obj = detect_obstacle(img_show_obj, out_boxes, dists/100., out_classes)
        save_path_obj = "%s/%s_obj.jpg" % (save_root, img_name.replace(".jpg", ""))
        img_show_obj = detect_single(img_src, img_show_obj, mtx, dist)
        #save_path_lane = "%s/%s_lane.jpg" % (save_root, img_name.replace(".jpg", ""))
        t = (time.time() - st)*1000
        print("[%d]--time: %.2f ms"%(i, t))
        cv2.imwrite(save_path_obj, img_show_obj)
    ave_time = (time.time()-total_st)/len(img_paths)*1000
    #print("average time: %.2f ms" % ave_time)

        #cv2.imwrite(save_path_lane, img_show_lane)


def detect_obstacle(img_view, boxes, dist, cls):
    n_object = dist.shape[0]
    if (n_object == 0):
        return img_view
    centers = np.zeros([n_object, 2])
    centers[:, 0] = (boxes[:, 0] + boxes[:, 2]) / 2
    centers[:, 1] = (boxes[:, 1] + boxes[:, 3]) / 2
    for i in range(n_object):
        if dist[i] == -1: continue
        if (dist[i] < 1.0):
            if (centers[i][0] <= 250) & (275 <= centers[i][1] <= 405) | (centers[i][0] > 250) & (
                    (45 - 275) * (centers[i][0] - 250) / (480 - 250) + 275 <= centers[i][1] <= (595 - 405) * (
                    centers[i][0] - 405) / (480 - 250) + 405):
                img_view = add_sign(img_view, cls[i])
    return img_view


def add_sign(img_view, cls_obj):
    class_names = dict({0: "stop", 1: "obstacle", 2: "slow", 3: "car"})
    sign = resize_img(cv2.imread("/home/pro/Documents/CAR/src/YAD2K/Pannal_Obstacle.jpg"), width=60, height=60)
    img_view[10:10 + sign.shape[0], 480:480 + sign.shape[1], :] = sign[:, :, :]
    img_view[10 + sign.shape[0]:35 + sign.shape[0], 480:480 + sign.shape[1], :] = 255
    font = cv2.FONT_HERSHEY_SIMPLEX
    img_view = cv2.putText(img_view, class_names[cls_obj], (480, 30 + sign.shape[0]), font, 0.6, (0, 0, 0), 2)
    return img_view




if __name__ == "__main__":

    print("loading model...")
    predictor = Predictor()
    rospy.spin()
