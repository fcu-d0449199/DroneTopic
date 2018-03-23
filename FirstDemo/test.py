import cv2
from feature_matching import FeatureMatching

#goal image
img_train = cv2.imread('./9.jpg')

#camera's image
query_image = cv2.imread('./3.jpg')
matching = FeatureMatching(query_image='./3.jpg')

m, flag = matching.match(img_train, query_image)
if(m == False): print("not match!!!")