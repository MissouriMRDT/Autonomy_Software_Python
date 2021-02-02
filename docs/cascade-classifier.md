# Process of training opencv classifier for own data set

Download opencv files from https://sourceforge.net/projects/opencvlibrary/files/3.4.13/ (the first .exe if on windows)

Resources:
https://docs.opencv.org/2.4/doc/user_guide/ug_traincascade.html
https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_objdetect/py_face_detection/py_face_detection.html

Sample commands used:

.\opencv_createsamples -img marker.jpg -bg bg.txt -info samplePos2/pos2.txt -maxxangle 0.5 -maxyangle 0.5 -maxzangle 0.5 -bgcolor 170 -bgthresh 0 -num 1000

.\opencv_createsamples -info samplePos2/pos2.txt -num 1000 -w 30 -h 30 -vec positives2.vec

.\opencv_traincascade -data data -vec final.vec -bg bg.txt -numPos 2700 -numNeg 1200 -numStages 10 -w 30 -h 30 -minHitRate 0.995 -maxFalseAlarmRate 0.3

-numPos needs to be slightly less than the actual number you have


.\opencv_createsamples -img marker2.png -bg bg.txt -info pos2/pos.txt -maxxangle 0.5 -maxyangle 0.5 -maxzangle 0.5 -bgcolor 170 -bgthresh 0 -num 4647

.\opencv_createsamples -info pos/pos.txt -num 4647 -w 20 -h 20 -vec positives20.vec

.\opencv_traincascade -data data20 -vec positives20.vec -bg bg.txt -numPos 4500 -numNeg 2250 -numStages 10 -w 20 -h 20 -minHitRate 0.995 -maxFalseAlarmRate 0.3