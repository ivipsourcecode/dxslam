import tensorflow as tf
import numpy as np
import os
import cv2
from tensorflow.python.saved_model import tag_constants
import sys
tf.contrib.resampler

class HFNet:
    def __init__(self, model_path, outputs):
        self.session = tf.Session()
        self.image_ph = tf.placeholder(tf.float32, shape=(None, None, 3))

        net_input = tf.image.rgb_to_grayscale(self.image_ph[None])
        tf.saved_model.loader.load(
            self.session, [tag_constants.SERVING], str(model_path),
            clear_devices=True,
            input_map={'image:0': net_input})

        graph = tf.get_default_graph()
        self.outputs = {n: graph.get_tensor_by_name(n + ':0')[0] for n in outputs}
        self.nms_radius_op = graph.get_tensor_by_name('pred/simple_nms/radius:0')
        self.num_keypoints_op = graph.get_tensor_by_name('pred/top_k_keypoints/k:0')

    def inference(self, image, nms_radius=4, num_keypoints=1000):
        inputs = {
            self.image_ph: image[..., ::-1].astype(np.float),
            self.nms_radius_op: nms_radius,
            self.num_keypoints_op: num_keypoints,
        }
        return self.session.run(self.outputs, feed_dict=inputs)

if __name__ == "__main__":
    if(len(sys.argv) != 3):
        print("please input the correct args")
        print("args1: image folder;")
        print("args1: featuer folder;")
        exit(0)

    #folders
    imageFolder = sys.argv[1]
    featuerFolder = sys.argv[2]

    #define the net
    model_path = "./model/hfnet"
    outputs = ['global_descriptor', 'keypoints', 'local_descriptors']
    hfnet = HFNet(model_path, outputs)

    #input the image
    imageNames = os.listdir(imageFolder)
    imageNames.sort()

    #create the output folder
    localDesFolder = os.path.join(featuerFolder, 'des')
    globalDesFolder = os.path.join(featuerFolder, 'glb')
    keypointFolder = os.path.join(featuerFolder, 'point-txt')
    if not os.path.exists(featuerFolder):
        os.mkdir(featuerFolder)
    if not os.path.exists(localDesFolder):
        os.mkdir(localDesFolder)
    if not os.path.exists(globalDesFolder):
        os.mkdir(globalDesFolder)
    if not os.path.exists(keypointFolder):
        os.mkdir(keypointFolder)

    #inference
    for imageName in imageNames:
        image = cv2.imread(os.path.join(imageFolder , imageName))
        query = hfnet.inference(image)
        print(imageName)
        localDes = np.asarray(query['local_descriptors'])
        np.save(os.path.join(localDesFolder , imageName.split(".png")[0]), localDes)
        globalDes = np.asarray(query['global_descriptor'])
        np.save(os.path.join(globalDesFolder, imageName.split(".png")[0]), globalDes)
        localIndex = np.asarray(query['keypoints'])
        np.savetxt(os.path.join(keypointFolder , imageName.split(".png")[0] + ".txt"), localIndex)

    print("hello")