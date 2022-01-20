# ROS


#### 工作空间(Catkin文件系统)

工作空间（workspace）是一个存放工程开发相关文件的文件夹。由4部分组成：

- src：代码空间（Source Space）存放ROS的catkin软件包（源代码包）；
- devel：开发空间（Development Space）生成的目标文件（包括头文件，动态链接库，静态链接库，可执行文件等）、环境变量；
- build：编译空间（Build Space）catkin（CMake）的缓存信息和中间文件；
- install：安装空间（Install Space）

```tex

workspace_folder/				-- WORKSPACE
	src/						-- SOURCE SPACE
		CMakeLists.txt			-- The 'toplevel' CMake file
		package_1/
			CMakeLists.txt
			package.xml
			...
		package_n/
			CMakeLists.txt
			package.xml
			...
	build/						-- BUILD SPACE
		CATKIN_IGNORE			-- Keeps catkin from walking directory
	devel/						-- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
		bin/
		etc/
		include/
		lib/
		share/
		.catkin
		env.bash
		setup.bash
		setup.sh
		...
	install/					-- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
		bin/
		etc/
		include/
		lib/
		share/
		.catkin
		env.bash
		setup.bash
		setup.sh
		...

```

- 创建工作空间

  ```bash
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws/src
  $ catkin_init_workspace
  ```

  空间名catkin_ws可自定义

- 编译工作空间

  ```bash
  $ cd ~/catkin_ws/
  $ catkin_make
  ```

- 设置环境变量

  ```bash
  $ source devel/setup.bash
  ```

- 检查环境变量

  ```bash
  $ echo $ROS_PACKAGE_PATH
  ```

  

##### src：代码空间（Source Space）

###### 功能包(Package)

创建功能包

```bash
$ catkin_creat-pkg<package_name>[depend1][depend2][depend3]
```

例如：

```bash
$ cd ~/caktin_ws/src
$ catkin_creat_pkg test_pkg std_msgs rospy roscpp
```

其中，"catkin_creat_pkg"为命令行；"test_pkg"为功能包名；"std_msgs""rospy""roscpp"为依赖项。常见的功能包包含(创建初始的功能包只含前四项)：

- **CMakeLists.txt：** package的编译规则；
- **package.xml：** package的描述信息；
- **include：** 放置C++源码对应的头文件；
- **src：** 存放源代码文件，包括C++的源码和(.cpp)以及Python的module(.py)；
- **msg：** 自定义消息(.msg)；
- **srv：** 自定义服务(.srv)；
- **urdf：** urdf文件(.urdf或.xacro)；
- **models：** 3D模型文件(.sda, .stl, .dae等)；
- **launch：** launch文件(.launch或.xml)；
- **scripts：** 可执行脚本，例如shell脚本(.sh)、Python脚本(.py)。

CMakeLists.txt 和 package.xml 是功能包必须具备的标志性文件。
{{<figure src="/images/17.png" title="创建好的功能包">}}

**package.xml** 文件中包括了package的描述信息：

- name, description, version, maintainer(s), license;
- opt. authors, url's, dependencies, plugins, etc...

**CMakeLists.txt** 文件由Cmake语言编写，构建package所需的CMake文件：

- 调用Catkin的函数/宏；
- 解析`package.xml ；
- 找到其他依赖的catkin软件包；
- 将本软件包添加到环境变量。

编译功能包：

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source~/catkin_ws/devel/setup.bash
```

> 同一个工作空间下，不允许存在同名功能包；不同工作空间下，允许存在同名功能包。

###### Package相关命令

**rospack：**

|        rostopic命令         |           作用            |
| :-------------------------: | :-----------------------: |
|       `rospack help`        |     显示rospack的用法     |
|       `rospack list`        |    列出本机所有package    |
| `rospack depends [package]` |    显示package的依赖包    |
|  `rospack find [package]`   |      定位某个package      |
|      `rospack profile`      | 刷新所有package的位置记录 |

**roscd：**

|     rosls命令     |        作用         |
| :---------------: | :-----------------: |
| `rosls [pacakge]` | 列出pacakge下的文件 |

**rosdep：**

|         rosdep命令         |            作用             |
| :------------------------: | :-------------------------: |
|  `rosdep check [pacakge]`  |  检查package的依赖是否满足  |
| `rosdep install [pacakge]` |      安装pacakge的依赖      |
|        `rosdep db`         |    生成和显示依赖数据库     |
|       `rosdep init`        | 初始化/etc/ros/rosdep中的源 |
|       `rosdep keys`        |  检查package的依赖是否满足  |
|      `rosdep update`       |   更新本地的rosdep数据库    |

一个较常使用的命令是`rosdep install --from-paths src --ignore-src --rosdistro=noetic -y`,用于安装工作空间中`src`路径下所有package的依赖项（由pacakge.xml文件指定）。

##### build：编译空间（Build Space）

##### devel：开发空间（Development Space）

##### install：安装空间（Install Space）

#### Catkin编译

catkin编译的工作流程如下：

1. 首先在工作空间 `catkin_ws/src/` 下递归的查找其中每一个ROS的package。
2. package中会有 `package.xml` 和 `CMakeLists.txt` 文件，Catkin(CMake)编译系统依据 `CMakeLists.txt` 文件,从而生成 `makefiles` (放在 `catkin_ws/build/` )。
3. 然后 `make` 刚刚生成的 `makefiles` 等文件，编译链接生成<u>可执行文件</u>(放在 `catkin_ws/devel` )。

使用 catkin_make 进行编译：

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash #刷新坏境
```

#### .launch 文件

###### \<launch\>

.launch文件中的根元素用\<launch\>标签定义

###### \<node\>

启动节点

```xml
<node pkg="package-name" type="executable-name" name="node-name" />
```

- pkg：节点所在的功能包名称
- type：节点的可执行文件名称
- name：节点运行时的名称
- output, respawn, required, ns, args 可选属性

###### \<param>/<rosparam\>

设置ROS系统运行中的参数，存储在参数服务器中；或者加载参数文件中的多个参数。

```xml
<param name="output_frame" value="odom" />
<rosparam file="params.yaml" command="load" ns="params" />
```

- name：参数名
- value：参数值

###### \<arg\>

launch文件内部的局部变量，仅限于launch文件使用。

```xml
<arg name="arg-name" default="arg-value" />
```

- name：参数名
- value：参数值

调用局部变量（使用美元符$）：

```xml
<param name="foo" value="$(arg arg-name)" />
<node name="node" pkg="package" type="type" args="$(arg arg-name)" />
```

> param 和 arg 都是参数，但存在位置不同。

###### \<remap\>

重映射ROS计算图资源的命名

```xml
<remap from="/turtlebot/cmd_vel"to="/cmd_vel" />
```

- from：原命名
- to：映射之后的命名

###### \<include\>

包含其他launch文件，类似C语言中的头文件包含。

```xml
<include file="$(dirname)/other.launch" />
```

- file：包含的其他launch文件的路径

.launch文件由roslaunch命令唤起，roslaunch命令详见[此处](http://wiki.ros.org/roslaunch)。

#### .urdf 文件

[链接](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch).

.urdf文件的基本格式：以标签 **\<robot\>\</robot\>**包裹整个文件；连接件用标签 **\<link\>\</link\>** 包裹；标签 **\<joint\>\</joint\>** 表示两个连接件之间的关系，内含标签 \<parent> 和 \<child\>；

```xml
<?xml version="1.0"?>
<robot name="test_robot">
  <link name="link1" />
  <link name="link2" />
  <link name="link3" />
  <link name="link4" />
 
  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
 
  <joint name="joint2" type="continuous">
    <parent link="link1"/>
    <child link="link3"/>
  </joint>
 
  <joint name="joint3" type="continuous">
    <parent link="link3"/>
    <child link="link4"/>
  </joint>
</robot>
```

{{<figure src="/images/21.png" title="joints和links">}}

**link** 的定义中可以添加其他标签，增加对连接件的描述：

- **\<collision\>：** 设置连杆的碰撞计算的信息；
- **\<visual\>：** 设置连接件的可视化信息；
- **\<inertial\>：** 设置连接件的惯性信息；
- **\<mass\>：** 设置重量（kg）；
- **\<inertia\>：** 设置惯性张量；
- **\<origin\>：** 设置相对于连接件的相对坐标系的移动和旋转；
- **\<geometry\>：** 设置模型的形状。可设置 box, cylindr, sphere 等形状， 也可在标签内直接导入COLLADA(.dae), STL(.stl)  等文件；
- **\<material\>：** 设置连接件的颜色和纹理等信息。

**joint** 的定义中可添加标签：

- **\<parent\>：** 关节的父连杆；
- **\<child\> ：** 关节的子连杆；
- **\<origin\>：** 将父连杆坐标系转换为子连杆坐标系；
- **\<axis\>：** 设置旋转轴；
- **\<limit\>：** 设置关节的速度、力和半径 （仅当关节是revolute或prismatic时）；

可定义类型(type)：

- **continuous：** 旋转关节，可以绕单轴无限旋转
- **revolute：** 旋转关节，类似于continuous，但旋转角度有限；
- **prismatic：** 滑动关节，沿某一轴线移动的关节，带有位置极限；
- **planar：** 平面关节，允许在平面正交方向上平移或者旋转；
- **floating：** 浮动关节，允许进行平移、旋转运动；
- **fiexd：** 固定关节，不允许运动的特殊关节；
- **calibration：** 关节参考位置，用来校准关节的绝对位置；
- **dynamics：** 用于描述物理属性，例如阻尼值、静摩擦力等；

例如：

```xml
<?xml version="1.0"?>
<robot name="test_robot">
	<link name="base_link">
		<visual>
			<geometry>
				<cylinder length="0.6" radius="0.2"/>
			</geometry>
		</visual>
	</link>

	<link name="link_1">
		<visual>
			<geometry>
				<box size="0.6 0.1 0.2"/>
			</geometry>
		</visual>
	</link>

	<joint name="base_to_link_1" type="fixed">
		<parent link="base_link"/>
		<child link="link_1"/>
	</joint>

</robot>
```

利用 roslaunch 命令在 rviz 中查看刚定义的模型：

```bash
sudo apt-get install ros-noetic-urdf-tutorial #安装urdf_tutorial包
roslaunch urdf_tutorial display.launch model:='$(filename)' #filename为urdf文件名
```

{{<figure src="/images/22.png" title="mutipleshapes">}}

所有的连接件均重合，因为其 origin 均相同。

###### \<origin\>

```xml
<origin xyz="x y z" rpy="r p y"
```

- **xyz：** 该连接件相对于父连接件(parent)的坐标位置，单位m；
- **rpy：** 该连接件相对于父连接件的角度位置，单位弧度(rad)；roll、pitch、yaw，翻滚、俯仰、偏航，对应x、y、z顺序。从坐标原点为基准，看向坐标轴正方向，绕坐标轴顺时针为正，逆时针为负。

\<origin\>标签包含在标签\<visual\>下。

>注意在\<link\>中和在\<joint\>中使用\<origin\>的不同。

示例：

```xml
<?xml version="1.0"?>
<robot name="test_robot">
	<link name="base_link">
		<visual>
			<geometry>
				<cylinder length="0.6" radius="0.2"/>
			</geometry>
		</visual>
	</link>

	<link name="link_1">
		<visual>
			<geometry>
				<box size="0.6 0.1 0.2"/>
			</geometry>
        <origin rpy="1.57075 1.57075 0" xyz="0 0 -0.7"/>
		</visual>
	</link>

	<joint name="base_to_link_1" type="fixed">
		<parent link="base_link"/>
		<child link="link_1"/>
        <origin rpy="1.57075 1.57075 0" xyz="0 0 -0.7"/>    
	</joint>

</robot>
```
{{<figure src="/images/23.png" title="origins">}}

###### \<material\>

```xml
<!--定义-->
<material name="blue">
	<color rgba="0 0 0.8 1"/>
</material>
<!--引用-->
<link name="base_link">
	<visual>
		<geometry>
			<cylinder length="0.6" radius="0.2"/>
		</geometry>
		<material name="blue"/>
	</visual>
</link>
```

###### 图形化显示 .urdf 文件

```bash
urdf_to_graphiz '$(filename.urdf(.xacro))'
evince '$(filenmae.pdf)'
```


