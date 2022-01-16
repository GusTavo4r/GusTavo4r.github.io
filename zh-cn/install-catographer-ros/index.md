# 安装CatographerROS




ROS文档：[Compiling Cartographer ROS — Cartographer ROS documentation (google-cartographer-ros.readthedocs.io)](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation)

**报错[1]:**

```bash
-- This workspace overlays: /opt/ros/noetic -- Found PythonInterp: /home/zonlin/Anaconda/ENTER/bin/python3 (found suitable version "3.8.8", minimum required is "3") -- Using PYTHON_EXECUTABLE: /home/zonlin/Anaconda/ENTER/bin/python3 -- Using Debian Python package layout -- Could NOT find PY_em (missing: PY_EM) CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message): Unable to find either executable 'empy' or Python module 'em'...  try installing the package 'python3-empy' Call Stack (most recent call first): /opt/ros/noetic/share/catkin/cmake/all.cmake:164 (include) /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:20 (include) CMakeLists.txt:24 (find_package)

-- Configuring incomplete, errors occurred! See also "/home/zonlin/catkin_ws/build_isolated/cartographer_ros_msgs/CMakeFiles/CMakeOutput.log". <== Failed to process package 'cartographer_ros_msgs': Command '['/home/zonlin/catkin_ws/install_isolated/env.sh', 'cmake', '/home/zonlin/catkin_ws/src/cartographer_ros/cartographer_ros_msgs', '-DCATKIN_DEVEL_PREFIX=/home/zonlin/catkin_ws/devel_isolated/cartographer_ros_msgs', '-DCMAKE_INSTALL_PREFIX=/home/zonlin/catkin_ws/install_isolated', '-G', 'Ninja']' returned non-zero exit status 1.

Reproduce this error by running: ==> cd /home/zonlin/catkin_ws/build_isolated/cartographer_ros_msgs && /home/zonlin/catkin_ws/install_isolated/env.sh cmake /home/zonlin/catkin_ws/src/cartographer_ros/cartographer_ros_msgs -DCATKIN_DEVEL_PREFIX=/home/zonlin/catkin_ws/devel_isolated/cartographer_ros_msgs -DCMAKE_INSTALL_PREFIX=/home/zonlin/catkin_ws/install_isolated -G Ninja
```

解决：(环境-Anaconda python3)

```bash
conda install -c conda-forge empy
```



**报错[2]:**

```bash
ImportError: "from catkin_pkg.package import parse_package" failed: No module named 'catkin_pkg' Make sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH. CMake Error at /opt/ros/noetic/share/catkin/cmake/safe_execute_process.cmake:11 (message): execute_process(/home/zonlin/Anaconda/ENTER/bin/python3 "/opt/ros/noetic/share/catkin/cmake/parse_package_xml.py" "/opt/ros/noetic/share/catkin/cmake/../package.xml" "/home/zonlin/catkin_ws/build_isolated/cartographer_ros_msgs/catkin/catkin_generated/version/package.cmake") returned error code 1
```

解决：

没有包就下载包：

```bash
conda install catkin_pkg
```

注意"Make sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH."

```bash
locate catkin_pkg
->usr/lib/python3/dist-packages/catkin_pkg
```

(安装locate)

```bash
sudo apt-get install mlocate
sudo updatedb
```

```bash
echo $PYTHONPATH
->opt/ros/noetic/lib/python3/dist-packages
```

需要的包catkin_pkg不在目录opt/ros/noetic/lib/python3/dist-packages下，在usr/lib/python3/dist-packages/catkin_pkg

```bash
vim ~/.bashrc
```

末尾添加：`export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages`

```bash
source ~/.bashrc
echo $PYTHONPATH
```

**[报错3]: **`catkin_make_isolated --install --use-ninja`

```bash
-- Build type: Release
CMake Warning at /usr/src/googletest/googlemock/CMakeLists.txt:43 (project):
  VERSION keyword not followed by a value or was followed by a value that expanded to nothing.
CMake Warning at /usr/src/googletest/googletest/CMakeLists.txt:54 (project):
  VERSION keyword not followed by a value or was followed by a value that expanded to nothing.
  CMake Error: The following variables are used in this project, but they are set to NOTFOUND.
Please set them or make sure they are set and tested correctly in the CMake files:

GMOCK_LIBRARY

  linked by target "time_conversion_test" in directory /home/zonlin/catkin_ws/src/cartographer_ros/cartographer_ros
  linked by target "configuration_files_test" in directory /home/zonlin/catkin_ws/src/cartographer_ros/cartographer_ros
  linked by target "msg_conversion_test" in directory /home/zonlin/catkin_ws/src/cartographer_ros/cartographer_ros
  linked by target "metrics_test" in directory /home/zonlin/catkin_ws/src/cartographer_ros/cartographer_ros

-- Configuring incomplete, errors occurred!
See also "/home/zonlin/catkin_ws/build_isolated/cartographer_ros/CMakeFiles/CMakeOutput.log".
See also "/home/zonlin/catkin_ws/build_isolated/cartographer_ros/CMakeFiles/CMakeError.log".

<== Failed to process package 'cartographer_ros': 
  Command '['/home/zonlin/catkin_ws/install_isolated/env.sh', 'cmake', '/home/zonlin/catkin_ws/src/cartographer_ros/cartographer_ros', '-DCATKIN_DEVEL_PREFIX=/home/zonlin/catkin_ws/devel_isolated/cartographer_ros', '-DCMAKE_INSTALL_PREFIX=/home/zonlin/catkin_ws/install_isolated', '-G', 'Ninja']' returned non-zero exit status 1.
```

原因是 GMOCK_LIBRARY 包缺失(7~15行)，解决：

```
sudo apt install libgmock-dev
```

**[报错4]:** `catkin_make_isolated --install --use-ninja`

```bash
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
Could not find a package configuration file provided by "robotnik_msgs"with any of the following names:

robotnik_msgsConfig.cmake
robotnik_msgs-config.cmake

Add the installation prefix of "robotnik_msgs" to CMAKE_PREFIX_PATH or set "robotnik_msgs_DIR" to a directory containing one of the above files.  If "robotnik_msgs" provides a separate development package or SDK, be sure it has been installed.

Call Stack (most recent call first):
summit_xl/summit_xl_common/summit_xl_pad/CMakeLists.txt:4 (find_package)
```

解决：src中的 summit_xl 包出了问题，删掉。

