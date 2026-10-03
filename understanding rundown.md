program.cpp file creates ROS node
within program.cpp, yolo detects objects, opencv annotates
program.cpp publishes annotated images and messages

yolo model should be focussed entirely on person / tent / object
in the case of future requirements, these parameters can be changed

"sahi detection" is not a model, but rather a way of subdividing a larger high quality image into smaller images such that they can be processed individually to get more infomation from them
each slice of the original image is run through yolo

==ros lifecycle node vs. ros plain node==

![[rundown#Major packages / tools used across the package]]

