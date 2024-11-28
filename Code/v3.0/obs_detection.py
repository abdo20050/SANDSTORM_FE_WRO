#!/usr/bin/env python


import cv2
import os
import sys
import getopt
from edge_impulse_linux.image import ImageImpulseRunner
from picamera2 import Picamera2
import numpy as np
runner = None
modelfile = "/home/jojo/model.eim"
runner = ImageImpulseRunner(modelfile)
model_info = runner.init()

def get_obstical_cordinates(frame):
    """
        code to detect obsticales using tinyML
        input: frame
        output: array of (label, x, y) for each obstical detected
    
    """
    obs_cord = []
    # model_info = runner.init(debug=True) # to get debug print out

    # print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
    labels = model_info['model_parameters']['labels']
    img = frame

    # imread returns images in BGR format, so we need to convert to RGB
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # get_features_from_image also takes a crop direction arguments in case you don't have square images
    # features, cropped = runner.get_features_from_image(img)

    # this mode uses the same settings used in studio to crop and resize the input
    features, cropped = runner.get_features_from_image_auto_studio_setings(img)
    for i in range(5):
        res = runner.classify(features)
        if len(res) == 0:
            return obs_cord # return empty
    print(img.shape)
    ratio = int(img.shape[0]//cropped.shape[0])
    
    # print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
    for bb in res["result"]["bounding_boxes"]:
        if bb['value'] < 0.993:
            continue
        # print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
        # cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)
        real_x = int(bb['x'])*ratio+img.shape[0]//2
        real_y = int(bb['y'])*ratio
        real_width = int(bb['width'])*ratio
        real_hight = int(bb['height'])*ratio
        
        img = cv2.rectangle(img, (real_x, real_y), (real_x + real_width , real_y + real_hight), (255, 0, 0), 1)
        x,y = (real_x + real_width//2, real_y + real_hight)
        classes = {'green_block': 0,
                 'red_block': 1,
                 'magenta_block':2}
        obs_cord.append([classes[bb['label']], x, y])
        # the image will be resized and cropped, save a copy of the picture here
        # so you can see what's being passed into the classifier
    obs_cord = np.array(obs_cord)
    return obs_cord


def main():
    # Main camera loop
    cap = Picamera2()
    RESIZE_FACTOR = 10
    cam_res = (cap.sensor_resolution[0] // RESIZE_FACTOR, cap.sensor_resolution[1] // RESIZE_FACTOR)
    video_config = cap.create_video_configuration(main={"format": "RGB888", "size": cam_res})
    cap.configure(video_config)
    cap.start()



    modelfile = "/home/jojo/model.eim"

    print('MODEL: ' + modelfile)

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            # model_info = runner.init(debug=True) # to get debug print out

            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']
            while(True):
                img = cap.capture_array()

                # imread returns images in BGR format, so we need to convert to RGB
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                # get_features_from_image also takes a crop direction arguments in case you don't have square images
                # features, cropped = runner.get_features_from_image(img)

                # this mode uses the same settings used in studio to crop and resize the input
                features, cropped = runner.get_features_from_image_auto_studio_setings(img)

                res = runner.classify(features)

                ratio = int(img.shape[0]//cropped.shape[0])
                
                print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                for bb in res["result"]["bounding_boxes"]:
                    print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                    cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)
                    real_x = np.array(bb['x'])*ratio+img.shape[0]//2
                    real_y = np.array(bb['y'])*ratio
                    img = cv2.rectangle(img, (real_x, real_y), (real_x + np.array(bb['width'])*ratio , real_y + np.array(bb['height'])*ratio), (255, 0, 0), 1)

                # the image will be resized and cropped, save a copy of the picture here
                # so you can see what's being passed into the classifier
                cv2.imshow('debug.jpg', cv2.cvtColor(cropped, cv2.COLOR_RGB2BGR))

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            if (runner):
                runner.stop()

if __name__ == "__main__":
    main()
    # cap = Picamera2()
    # RESIZE_FACTOR = 10
    # cam_res = (cap.sensor_resolution[0] // RESIZE_FACTOR, cap.sensor_resolution[1] // RESIZE_FACTOR)
    # video_config = cap.create_video_configuration(main={"format": "RGB888", "size": cam_res})
    # cap.configure(video_config)
    # cap.start()
    # while(True):
        
    #     frame = cap.capture_array()
    #     obs_cord = get_obstical_cordinates(frame)
    #     print(obs_cord)