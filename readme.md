# Depth Clustering #

[![Build Status](https://travis-ci.org/niosus/depth_clustering.svg?branch=github_travis)](https://travis-ci.org/niosus/depth_clustering)

This is a fast and robust algorithm to segment point clouds taken with
Velodyne sensor into objects. It works with all available Velodyne sensors,
i.e. 16, 32 and 64 beam ones.

## How to build? ##
### Prerequisites ###
- OpenCV: `sudo apt-get install libopencv-dev`
- QGLViewer and Qt4:
    + `sudo apt-get install libqglviewer-dev`
    + `sudo apt-get install libqt4-dev`
- (optional) PCL - needed for saving clouds node
- (optional) ROS - needed for subscribing to topics

### Build script  ###
- `mkdir build`
- `cd build`
- `cmake ..`
- `make -j4`
- (optional) `ctest -VV`

It can also be built as a ROS package with:
- `catkin build depth_clustering` if the code is inside catkin workspace.

P.S. in case you don't use `catking build` you [should][catkin_tools_docs].
Install it by `sudo apt-get install python-catkin-tools`.

## How to run? ##
See [examples](examples/). There are ROS nodes as well as standalone
binaries. Examples include showing axis oriented bounding boxes around found
objects (these start with `show_objects_` prefix) as well as a node to save all
segments to disk. The examples should be easy to tweak for your needs.

## Run on real world data ##
Go to folder with binaries:
```
cd <path_to_project>/build/devel/lib/depth_clustering
```

#### Frank Moosman Velodyne SLAM - Dataset ####
Get the data:
```
mkdir data/; wget http://www.mrt.kit.edu/z/publ/download/velodyneslam/data/scenario1.zip -O data/moosman.zip; unzip data/moosman.zip -d data/; rm data/moosman.zip
```

Run a binary to show detected objects:
```
./show_objects_moosman --path data/scenario1/
```

#### Other data ####
There are also examples on how to run the processing on KITTI data and on ROS
input. Follow the `--help` output of each of the examples for more details.

## Documentation ##
You should be able to get Doxygen documentation by running:
```
cd doc/
doxygen Doxyfile.conf
```

## Related publications ##
Please cite related paper if you use this code:

```
@InProceedings{bogoslavskyi16iros,
Title = {Fast Range Image-Based Segmentation of Sparse 3D Laser Scans for Online Operation},
Author = {I. Bogoslavskyi and C. Stachniss},
Booktitle = {Proc. of The International Conference on Intelligent Robots and Systems (IROS)},
Year = {2016},
Url = {http://www.ipb.uni-bonn.de/pdfs/bogoslavskyi16iros.pdf}
}
```


[build-status-img]: https://gitlab.ipb.uni-bonn.de/igor/depth_clustering/badges/master/build.svg
[coverage-img]: https://gitlab.ipb.uni-bonn.de/igor/depth_clustering/badges/master/coverage.svg
[commits-link]: https://gitlab.ipb.uni-bonn.de/igor/depth_clustering/commits/master

[catkin_tools_docs]: https://catkin-tools.readthedocs.io/en/latest/installing.html
