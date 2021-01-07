# Navigating - Traversing from one GPS waypoint to another
# Search Pattern - Looking for AR Tag while traveling in a spiral
# Approaching Marker- Track the AR Tag, keep it in the center of the screen, approach until we think we are 1.5 meters away
import numpy as np
import cv2
import os
import urllib.request

# Databases
# http://image-net.org/api/text/imagenet.synset.geturls?wnid=n09270735
# http://image-net.org/api/text/imagenet.synset.geturls?wnid=n12284262
# http://image-net.org/api/text/imagenet.synset.geturls?wnid=n09287968


def grab_negative_images():
    neg_images_link = "http://image-net.org/api/text/imagenet.synset.geturls?wnid=n09270735"
    neg_image_urls = urllib.request.urlopen(neg_images_link, timeout=5).read().decode()
    pic_num = 1261

    if not os.path.exists("neg"):
        os.makedirs("neg")

    for i in neg_image_urls.split("\n"):
        try:
            print(i)
            urllib.request.urlretrieve(i, "neg/" + str(pic_num) + ".jpg")
            img = cv2.imread("neg/" + str(pic_num) + ".jpg", cv2.IMREAD_GRAYSCALE)
            # should be larger than samples / pos pic (so we can place our image on it)
            resized_image = cv2.resize(img, (720, 480))
            cv2.imwrite("neg/" + str(pic_num) + ".jpg", resized_image)
            pic_num += 1

        except Exception as e:
            print(str(e))


def create_pos_n_neg():
    for file_type in ["neg", "pos"]:
        for img in os.listdir(file_type):
            if file_type == "pos":
                line = file_type + "/" + img + " 1 0 0 50 50\n"
                with open("info.dat", "a") as f:
                    f.write(line)
            elif file_type == "neg":
                line = file_type + "/" + img + "\n"
                with open("bg.txt", "a") as f:
                    f.write(line)


def grayscale():
    img = cv2.imread("marker.png", cv2.IMREAD_GRAYSCALE)
    cv2.imwrite("marker2.png", img)


def classify():
    cap = cv2.VideoCapture("ar.avi")

    while cap.isOpened():
        ret, frame = cap.read()
        tag_cascade = cv2.CascadeClassifier("cascade.xml")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = tag_cascade.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in tags:
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        cv2.imshow("img", frame)
        cv2.imshow("gray", gray)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    classify()


def find_ar_tag():
    """"""
    return ar_tag


def track_ar_tag():
    """"""
    return
