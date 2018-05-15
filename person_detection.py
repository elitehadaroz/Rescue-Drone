import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import time
import socket
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
import cv2
from utils import label_map_util
from utils import visualization_utils as vis_util
import tkinter as tk
from PIL import Image, ImageTk
import threading
from multiprocessing import Process, Queue
import queue
import pickle
import struct




cap=cv2.VideoCapture(0)


sys.path.append("..")



os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
# What model to download.
MODEL_NAME = 'person_deection'
#MODEL_FILE = MODEL_NAME + '.tar.gz'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')

NUM_CLASSES = 1


session_conf = tf.ConfigProto(
    device_count={'CPU' : 1, 'GPU' : 0},
    allow_soft_placement=True,
    log_device_placement=False
)

detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


# ## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine



label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


# ## Helper code

"""
def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)
"""

# # Detection



# image1.jpg
# image2.jpg
#PATH_TO_TEST_IMAGES_DIR = 'test_images'
#TEST_IMAGE_PATHS = [ os.path.join(PATH_TO_TEST_IMAGES_DIR, 'image{}.jpg'.format(i)) for i in range(1, 3) ]

# Size, in inches, of the output images.
#IMAGE_SIZE = (12, 8)
clientsocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('localhost',8080))
#pickle.dump(protocol=2)
def main():
  with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
      # Definite input and output Tensors for detection_graph
      image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
      detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
      detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
      while True:
        ret,image_np=cap.read()
        if ret:
          #image_np = cv2.flip(image_np, 1)
          #image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        
          image_np_expanded = np.expand_dims(image_np, axis=0)
          # Actual detection.
          (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})
        
          #Visualization of the results of a detection.
          name_type =[category_index.get(value) for index,value in enumerate(classes[0]) if scores[0,index] > 0.5]
          if(len(name_type) !=0 ):
            if name_type[0] is not None :
              MESSAGE=(name_type[0]['name']) #name of type detection
              sys.stdout.write(MESSAGE+ "\n")
              #print(MESSAGE)
              sys.stdout.flush()
              #sock.sendto(MESSAGE.encode('utf-8'), (UDP_IP, UDP_PORT))   
              if name_type[0]['name'] == "person":
                vis_util.visualize_boxes_and_labels_on_image_array(
              image_np,
              np.squeeze(boxes),
              np.squeeze(classes).astype(np.int32),
              np.squeeze(scores),
              category_index,
              use_normalized_coordinates=True,
              line_thickness=8)
          #if(category_index.get(value) for index in enumerate(classes[0]) == "person"):
          #cv2.imshow('object detection', cv2.resize(image_np, (480,640)))
          data = pickle.dumps(image_np,protocol=1) ### new code
          try:
            clientsocket.sendall(struct.pack("L", len(data)) + data)  ### new code
          except socket.error:
            print("the video break from person detection")
            break
            #clientsocket.shutdown(socket.SHUT_RDWR)
          #img = Image.fromarray(image_np)
          #imgtk = ImageTk.PhotoImage(image=img)
          #lmain.imgtk = imgtk
          #lmain.configure(image=imgtk)
          #queue.put(image_np) 
          if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    
if __name__ == "__main__":
  #Set up GUI
  #queue = queue.Queue()
  #window = tk.Tk()  #Makes main window
  #window.wm_title("Digital Microscope")
  #window.config(background="#FFFFFF")
  main()
  #lmain = tk.Label(window)
  #lmain.grid(row=1, column=1)
  #vid =threading.Thread(name='vid',target=main)
  #vid.start()
  #drone =  "cd C:\Python27 python drone_control.py"  # launch the person_detection script using bash
  #droneProcess = subprocess.Popen(drone,shell=False, stdout=subprocess.PIPE)

  #s =threading.Thread(name='vid',target=read)
  #s.start()
  #window.mainloop()
