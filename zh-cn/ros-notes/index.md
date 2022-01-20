# ROS


#### 工作空间

工作空间（workspace）是一个存放工程开发相关文件的文件夹。由4部分组成：

- src：代码空间（Source Space）
- devel：开发空间（Development Space）
- build：编译空间（Build Space）
- install：安装空间（Install Space）

```

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

###### 创建功能包

```bash
$ catkin_creat-pkg<package_name>[depend1][depend2][depend3]
```

创建功能包

```bash
$ cd ~/caktin_ws/src
$ catkin_creat_pkg test_pkg std_msgs rospy roscpp
```

其中，"catkin_creat_pkg"为命令行；"test_pkg"为功能包名；"std_msgs""rospy""roscpp"为依赖项。创建好的功能包包含：

- include：放置头文件
- src：放置代码
- CMakeLists.txt
- package.xml

CMakeLists.txt 和 package.xml 是功能包必须具备的标志性文件。
{{<figure src="/images/17.png" title="创建好的功能包">}}

package.xml文件中：
{{<figure src="/images/18.png" title="package.xml文件中的基本信息">}}
{{<figure src="/images/19.png" title="package.xml文件中的依赖项">}}

CMakeLists.txt文件由Cmake语言编写，其中也包含了依赖项：
{{<figure src="/images/20.png" title="CMakeLists.txt中的依赖项">}}
编译功能包

```bash
$ cd ~/catkin_ws
$ catkin_make
$source~/catkin_ws/devel/setup.bash
```

> 同一个工作空间下，不允许存在同名功能包；不同工作空间下，允许存在同名功能包。

##### build：编译空间（Build Space）

##### devel：开发空间（Development Space）

##### install：安装空间（Install Space）

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

```xml
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
