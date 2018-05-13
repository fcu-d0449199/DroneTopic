import cv2
from feature_matching import FeatureMatching

#camera's image
img_train = cv2.imread('./2.jpg')

#goal image
query_image = cv2.imread('./1.jpg')
matching = FeatureMatching(query_image='./1.jpg')

m, flag = matching.match(img_train, query_image)
if(m == False): print("not match!!!")
