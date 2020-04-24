# Intro to Gazebo World Modeling

## Install Mercurial 
```
sudo apt install mercurial
```

## Add More Models

Use Mercurial to get a bunch of open source gazebo models from the Open Source Robotics Foundation (OSRF) 

```
hg clone https://bitbucket.org/osrf/gazebo_models ~/gazebo_ws/gazebo_models
```
Add Models path to the bashrc
```
echo 'export GAZEBO_MODEL_PATH=~/gazebo_ws/gazebo_models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
source ~/.bashrc
```

## A New Drone Simulation World

make a new file in `~/catkin_ws/src/iq_sim/worlds/` called `hills.world`

copy the following lines into the world 

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.9</erp>
          <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.0</contact_surface_layer>
        </constraints>
      </ode>
      <real_time_update_rate>-1</real_time_update_rate>
      <!-- <max_step_size>0.0020</max_step_size> -->
    </physics>

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="iris">
      <include>
        <uri>model://iris_with_standoffs_demo</uri>
      </include>






      <!-- add new camera -->
      <link name='camera'>
        <pose>0 -0.01 0.070 .8 0 1.57</pose>
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
        <visual name='visual'>
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>0.025</length>
            </cylinder>
          </geometry>
           <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>

        <sensor name="camera" type="camera">
          <pose>0 0 0 -1.57 -1.57 0</pose>
          <camera>
            <horizontal_fov>1.0472</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
            </image>
            <clip>
              <near>0.05</near>
              <far>1000</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>true</visualize>

         <!--  <plugin name="irlock" filename="libArduCopterIRLockPlugin.so">
              <fiducial>irlock_beacon_01</fiducial>
          </plugin> -->
          <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>0.0</updateRate>
          <cameraName>webcam</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>camera_link</frameName>
          <hackBaseline>0.07</hackBaseline>
          <distortionK1>0.0</distortionK1>
          <distortionK2>0.0</distortionK2>
          <distortionK3>0.0</distortionK3>
          <distortionT1>0.0</distortionT1>
          <distortionT2>0.0</distortionT2>
      </plugin>

        </sensor>

      </link>

      <!-- attach camera -->
      <joint type="revolute" name="base_camera_joint">
        <pose>0 0 0.0 0 0 0</pose>
        <parent>iris::iris_demo::gimbal_small_2d::tilt_link</parent>
        <child>camera</child>
        <axis>
          <limit>
            <lower>0</lower>
            <upper>0</upper>
          </limit>
          <xyz>0 0 1</xyz>
          <use_parent_model_frame>true</use_parent_model_frame>
        </axis>
      </joint>
    </model>

  </world>
</sdf>
```
The above code is `sdf` code. it is a formatting language specific to Gazebo. The above lines form the basis to our a good gazebo drone simulation. The physics tag above is tailored to best simulate the drone's flight. The next part of the simulation is the `iris` which is our drone. The code inside the iris model tag is the code needed for our camera sensor. We will go over sdf coding in future tutorials.

## Add a Launch File to Facilitate the ROS Plugins 

make a file in `~/catkin_ws/src/iq_sim/launch` called `hills.launch`

then add the following lines

```
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find iq_sim)/worlds/hills.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>
```
save the files ( `ctrl + s` )

## Launch the New World 

```
roslaunch iq_sim hills.launch
```

## Add Other Models 

click on the `Insert` tab in the left column 

add the model `Winding Valley Heightmap`

delete the model `Ground Plane`

add the model `standing person`

reposition the models as seen fit

save file and override our file hills.world

---

### References

http://gazebosim.org/tutorials
