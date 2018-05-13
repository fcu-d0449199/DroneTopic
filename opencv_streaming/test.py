import cv2
from feature_matching import FeatureMatching
import time

try:
    VideoStream = cv2.VideoCapture(0) # index of your camera
except:
    print "problem opening input stream"

times = 0
while(VideoStream.isOpened()):
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Capture frame-by-frame
    ret, frame = VideoStream.read()
    if ret == True:
        # Display the resulting frame
        cv2.imshow("camera's streaming video", frame)
        # write the frame
        timestr = time.strftime("%Y_%m%d-%H_%M_%S")
        ImageName = "image_" + timestr + "__" + str(times + 1) + ".jpg"
        cv2.imwrite(ImageName, frame)
        print("\n save picture" + str(times + 1) + "...\n")
        times += 1

        # camera's image
        img_train = cv2.imread('./' + ImageName)
        # goal image
        query_image = cv2.imread('./Goal.jpg')
        matching = FeatureMatching(query_image='./Goal.jpg')

        m = matching.match(img_train, query_image)
        if (m == False):
            print("not match!!!\n")

    else:
        break

# When everything done, release the capture
VideoStream.release()
cv2.destroyAllWindows()
