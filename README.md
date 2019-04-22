First of all, I am very grateful to rubengooj for the open source code.
I encountered a lot of problems during the make process, here to record the problems and solutions in the compilation process.		
1. opencv：
I ended up installing opencv3.2.0 and opencv_contrib-3.2.0.
https://github.com/opencv/opencv/tree/3.2.0		
https://github.com/opencv/opencv_contrib/tree/3.2.0
cmake:
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.2.0/modules/ ..
2. Eigen3：sudo apt-get install libeigen3-dev
3. boost：sudo apt-get install libboost-dev
You may encounter the following problems during the compilation process.
Cmake can't link boost to the following:
- boost_regex
- boost_thread
- boost_system
- boost_filesystem

First, you need to confirm if boost_system exists.If it exists, and  could not be found.
You should install  libboost-filesystem-dev.
	sudo apt-get install libboost-filesystem-dev
then install  libboost-regex-dev， libboost-thread-dev...，anyway，What you are missing：
	sudo apt-get install libboost-regex-dev
	sudo apt-get install libboost-thread-dev
4. g2o:	I use the newest version(https://github.com/RainerKuemmerle/g2o).
Then you could see that:
error: no matching function for call to ‘g2o::BlockSolver<g2o::BlockSolverTraits<6, 3> ...
error: no matching function for call to ‘g2o::OptimizationAlgorithmLevenberg::OptimizationAlgorithmLevenberg(g2o::BlockSolver_6_3&)’
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);	

You can change
g2o::BlockSolver_6_3::LinearSolverType* linearSolver;
linearSolver = new g2o::LinearSolverCholmodg2o::BlockSolver_6_3::PoseMatrixType();
g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

to

std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>());
std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));


5. MRPT: I use https://github.com/MRPT/mrpt/tree/0c3d605c3cbf5f2ffb8137089e43ebdae5a55de3.	

Notice that this repository is only an open-source version of PL-SLAM released with the aim of being useful for the community, however, it is far from being optimized and we are not including some features of PL-SLAM.

# PL-SLAM #

This code contains an algorithm to compute stereo visual SLAM by using both point and line segment features.

**Authors:** [Ruben Gomez-Ojeda](http://mapir.isa.uma.es/mapirwebsite/index.php/people/164-ruben-gomez), [David Zuñiga-Noël](http://mapir.isa.uma.es/mapirwebsite/index.php/people/270), [Francisco Angel Moreno](http://mapir.isa.uma.es/mapirwebsite/index.php/people/199-francisco-moreno-due%C3%B1as), [Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html), and [Javier Gonzalez-Jimenez](http://mapir.isa.uma.es/mapirwebsite/index.php/people/95-javier-gonzalez-jimenez)

**Related publication:** [*PL-SLAM: a Stereo SLAM System through the Combination of Points and Line Segments*](http://mapir.isa.uma.es/mapirwebsite/index.php/people/164-ruben-gomez)

If you use PL-SLAM in your research work, please cite:

    @article{gomez2017pl,
      title   = {{PL-SLAM: a Stereo SLAM System through the Combination of Points and Line Segments}},
      author  = {Gomez-Ojeda, Ruben and Zuñiga-Noël, David and Moreno, Francisco-Angel and Scaramuzza, Davide and Gonzalez-Jimenez, Javier},
      journal = {arXiv preprint arXiv:1705.09479},
      year    = {2017}
}

The pdf file can be found at [https://arxiv.org/abs/1705.09479](https://arxiv.org/abs/1705.09479).

[![PL-SLAM](https://img.youtube.com/vi/-lCTf_tAxhQ/0.jpg)](https://www.youtube.com/watch?v=-lCTf_tAxhQ)

**Related publications:**

[Gomez-Ojeda, R., Briales, J., & Gonzalez-Jimenez, J. (2016, October). PL-SVO: Semi-direct monocular visual odometry by combining points and line segments. In Intelligent Robots and Systems (IROS), 2016 IEEE/RSJ International Conference on (pp. 4211-4216). IEEE.](http://mapir.isa.uma.es/rgomez/publications/iros16plsvo.pdf)

[Gomez-Ojeda, R., & Gonzalez-Jimenez, J. (2016, May). Robust stereo visual odometry through a probabilistic combination of points and line segments. In Robotics and Automation (ICRA), 2016 IEEE International Conference on (pp. 2521-2526). IEEE.](http://mapir.isa.uma.es/rgomez/publications/icra16plsvo.pdf).

**License:**

The provided code is published under the General Public License Version 3 (GPL v3). More information can be found in the "LICENSE" also included in the repository.

Please do not hesitate to contact the authors if you have any further questions.


## 1. Prerequisites and dependencies

### OpenCV 3.x.x
It can be easily found at http://opencv.org. 

### Eigen3 (tested with 3.2.92)
http://eigen.tuxfamily.org

### Boost
Installation on Ubuntu:
```
sudo apt-get install libboost-dev
```

### g2o - General Graph Optimization
It can be found at:
```
https://github.com/RainerKuemmerle/g2o.git
```

### YAML (tested with 0.5.2)
Installation on Ubuntu:
```
sudo apt-get install libyaml-cpp-dev
```

### stvo-pl
It can be found at: 
```
https://github.com/rubengooj/stvo-pl
```

### MRPT
In case of using the provided representation class. 
Download and install instructions can be found at: http://www.mrpt.org/

#### Known Issues:
If working with the most recent versions of the MRPT library you might find some issues due to hard refactoring, for which we recommend to use this version instead (the last one we tested):
```
https://github.com/MRPT/mrpt/tree/0c3d605c3cbf5f2ffb8137089e43ebdae5a55de3
```

### Line Descriptor
We have modified the [*line_descriptor*](https://github.com/opencv/opencv_contrib/tree/master/modules/line_descriptor) module from the [OpenCV/contrib](https://github.com/opencv/opencv_contrib) library (both BSD) which is included in the *3rdparty* folder.


## 2. Configuration and generation

Executing the file *build.sh* will configure and generate the *line_descriptor* and *DBoW2* modules, uncompress the vocabulary files, and then will configure and generate the *PL-SLAM* library for which we generate: **libplslam.so** in the lib folder, and the application **plslam_dataset** that works with our dataset format (explained in the next section).


## 3. Usage

### Datasets configuration
We employ an environment variable, *${DATASETS_DIR}*, pointing the directory that contains our datasets. Each sequence from each dataset must contain in its root folder a file named *dataset_params.yaml*, that indicates at least the camera model and the subfolders with the left and right images. We provide dataset parameters files for several datasets and cameras with the format *xxxx_params.yaml*.

### Configuration files
For running SLAM we can load the default parameters file or employ the *config_xxxx.yaml* files provided for every dataset.

### SLAM Application
Usage: ./plslam_dataset <dataset_name> [options]
Options:
	-c Config file
	-o Offset (number of frames to skip in the dataset directory
	-n Number of frames to process the sequence
	-s Parameter to skip s-1 frames (default 1)

A full command would be:

./plslam_dataset kitti/00 -c ../config/config_kitti.yaml -o 100 -s 2 -n 1000

where we are processing the sequence 00 from the KITTI dataset (in our dataset folders) with the custom config file, with an offset *-c* allowing to skip the first 100 images, a parameter *-s* to consider only one every 2 images, and a parameter *-n* to only consider 1000 input pairs.

