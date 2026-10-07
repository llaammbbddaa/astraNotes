[OpenCV Tutorial in 5 minutes - All Modules Overview - YouTube](https://youtu.be/PeMM80WimN4?si=y1B-uz6XfJcONd-3)
- opencv is basically a massive library that contains like a billion different ways to annotate an image (or in our case, series of images)
- in regards to [[yolo]], opencv is basically just the container that prepares and presents the data in and out of yolo
	for example, yolo may only output this, but opencv would then take that information and overlay a box on the original image
```
tensor([[102.4, 48.7, 311.9, 590.2]])   # xyxy
tensor([0.91])                           # conf
tensor([0.])                             # cls
person                                   # name
```