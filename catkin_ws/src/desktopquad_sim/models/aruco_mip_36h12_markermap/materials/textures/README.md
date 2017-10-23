ArUco_MIP_36h12 5x7 Marker Map
==============================

In order to get a correctly sized marker map into Gazebo, the following steps were performed:

1. Create the marker map

	```bash
	# After building the aruco_2.0.19 library
	$ aruco_create_markermap 5:7 out.png out.yml -r 50
	```

1. "Print" (save to file) the `.png` to a `.pdf` using standard Linux Image Viewer. The purpose of this is to get the correct dimensions (and margin) of what a camera would actually see. (Although, empirically I have found that printing from this pdf and from the png do result in slightly different sizes)

1. Use `ImageMagick` to convert the `.pdf` back to a `.png`.

	```bash
	$ convert -density 288 in.png.pdf -background white -alpha remove out.png
	```

	The `-density` option specifies the DPI of the image. Note that `4*72 = 288`, so `out.png` will be a 72 DPI image. The `-background white -alpha remove` part removes the alpha channel and just replaces transparency with white (see [http://stackoverflow.com/a/8437562](http://stackoverflow.com/a/8437562)).

**Note:** To check the DPI of a current `.png`, you can run the follow command:

```bash
# see http://stackoverflow.com/q/14632254
$ identify -format "%w x %h %x x %y\n" in.png
```