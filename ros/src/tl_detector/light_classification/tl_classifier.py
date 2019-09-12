from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import numpy as np
import rospy

# only keep detections with high scores
def filter_boxes(score_threshold, det_boxes, det_scores, det_classes):
    n = len(det_classes)
    idxs = []
    for i in range(n):
        if det_scores[i] >= score_threshold:
            idxs.append(i)
 
    filtered_boxes = det_boxes[idxs]
    filtered_scores = det_scores[idxs]
    filtered_classes = det_classes[idxs]
    return filtered_boxes, filtered_scores, filtered_classes


class TLClassifier(object):
    def __init__(self):
        self.model = self.load_graph()
        self.model.as_default()
        self.sess = tf.Session(graph=self.model)
        self.image_tensor = self.model.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.det_boxes = self.model.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.det_scores = self.model.get_tensor_by_name('detection_scores:0')
        self.det_classes = self.model.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.model.get_tensor_by_name('num_detections:0')


    def load_graph(self):
        detection_graph = tf.Graph()
        print("Loading graph...")
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile('light_classifier.pb', 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        rospy.loginfo("Graph loaded!")
        return detection_graph

    def get_classification(self, image):
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)

        # Actual detection.
        (boxes, scores, classes, num_detections) = self.sess.run(
            [self.det_boxes, self.det_scores, self.det_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        score_threshold = 0.6
        boxes, scores, classes = filter_boxes(score_threshold, boxes, scores, classes)

        class_id = -1
        result = TrafficLight.UNKNOWN;
        if len(scores) > 0:
            class_id = classes[0]

            if class_id == 1 or class_id == 2:
                light_color = 'red'
                result = TrafficLight.RED
            elif class_id == 3:
                light_color = 'green'
                result = TrafficLight.GREEN
        

        return result


