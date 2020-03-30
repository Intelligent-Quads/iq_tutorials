# Adding a Sensor to a Gazebo Robot 

This tutorial will go how to add an existing gazebo sensor to an existing gazebo robot.

## Available Plugins List

To see a list of available gazebo sensor plugins take a look at this [link](http://gazebosim.org/tutorials?tut=ros_gzplugins#Pluginsavailableingazebo_plugins) 

## Add a 2d LiDAR to our Drone Model

There are 2 ways to add sensor to a model. The first is to modify a model's sdf file. The second is to add the sensor with  the `<model>` tags in side the world file. the second method is what we will focus on today.

Open the flie `runway.world` in the folder `iq_sim/wolrds`.  then scroll to the line 
```
<model name="iris">
```
this line is where our drone is specified.  You will notice a camera already attached to the drone. we are going to do a similar method to add our 2d lidar. 

we will then add the following lines below the `</joint>` tag for the camera. 

```
   <!--add lidar-->
        <link name="hokuyo_link">
          <pose>0 0 0 0 0 0</pose>
          <collision name="collision">
            <pose>0 0 0.3 0 0 0</pose>
            <geometry>
              <box>
                <size>0.1 0.1 0.1</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <pose>0 0 0.27 0 0 0</pose>
            <geometry>
              <mesh>
                <uri>model://hokuyo/meshes/hokuyo.dae</uri>
              </mesh>
            </geometry>
          </visual>
          <inertial>
            <mass>0.016</mass>
            <inertia>
               <ixx>0.0001</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.0001</iyy>
               <iyz>0</iyz>
               <izz>0.0001</izz>
               <!-- low intertia necessary to avoid not disturb the drone -->
            </inertia>
          </inertial>

          <sensor type="ray" name="laser">
            <pose>0 0 0.3 0 0 1.57</pose>
            <visualize>true</visualize>
            <update_rate>10</update_rate>
            <ray>
              <scan>
                <horizontal>
                  <samples>1024</samples>
                  <resolution>1</resolution>
                  <min_angle>-3.141593</min_angle>
                  <max_angle>3.141593</max_angle>
                </horizontal>
              </scan>
              <range>
                <min>0.1</min>
                <max>30</max>
                <resolution>0.1</resolution>
              </range>
              <!-- <noise>
                <type>Gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
              </noise> -->
            </ray>
            <plugin name="hokuyo_node" filename="libgazebo_ros_laser.so">
              <robotNamespace></robotNamespace>
              <topicName>/spur/laser/scan</topicName>
              <frameName>/hokuyo_sensor_link</frameName>
            </plugin>
          </sensor>
        </link>

        <joint name="hokuyo_joint" type="fixed">
          <pose>0 0 0 0 0 0</pose>
          <parent>iris::iris_demo::iris::base_link</parent>
          <child>hokuyo_link</child>
        </joint>

```

## Key Tags

- `<pose>` specify a position of an object with respect to the last object in the hierarchy

- `<link>` specifies an object to be attached to the model 

- `<joint>` specifies how the link is attached to the model


## Completed World File 

`runway.world` should look like this 

```
<?xml version="1.0"?> 
<sdf version="1.5">
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

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>5000 5000</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="runway">
          <pose>000 0 0.005 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1829 45</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Runway</name>
            </script>
          </material>
        </visual>

        <visual name="grass">
          <pose>0 0 -0.1 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>5000 5000</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grass</name>
            </script>
          </material>
        </visual>

      </link>
    </model>

    <include>
      <uri>model://sun</uri>
    </include>
    <model name="iris">
      <include>
        <uri>model://iris_with_standoffs_demo</uri>
      </include>
      <pose> 0 0 0 0 0 0</pose>


      <!-- add new camera -->
      <link name='camera'>
        <pose>0 -0.01 0.070 1.57 0 1.57</pose>
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



         <!--add lidar-->
        <link name="hokuyo_link">
          <pose>0 0 0 0 0 0</pose>
          <collision name="collision">
            <pose>0 0 0.3 0 0 0</pose>
            <geometry>
              <box>
                <size>0.1 0.1 0.1</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <pose>0 0 0.27 0 0 0</pose>
            <geometry>
              <mesh>
                <uri>model://hokuyo/meshes/hokuyo.dae</uri>
              </mesh>
            </geometry>
          </visual>
          <inertial>
            <mass>0.016</mass>
            <inertia>
               <ixx>0.0001</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.0001</iyy>
               <iyz>0</iyz>
               <izz>0.0001</izz>
               <!-- low intertia necessary to avoid not disturb the drone -->
            </inertia>
          </inertial>

          <sensor type="ray" name="laser">
            <pose>0 0 0.3 0 0 1.57</pose>
            <visualize>true</visualize>
            <update_rate>10</update_rate>
            <ray>
              <scan>
                <horizontal>
                  <samples>1024</samples>
                  <resolution>1</resolution>
                  <min_angle>-3.141593</min_angle>
                  <max_angle>3.141593</max_angle>
                </horizontal>
              </scan>
              <range>
                <min>0.1</min>
                <max>30</max>
                <resolution>0.1</resolution>
              </range>
              <!-- <noise>
                <type>Gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
              </noise> -->
            </ray>
            <plugin name="hokuyo_node" filename="libgazebo_ros_laser.so">
              <robotNamespace></robotNamespace>
              <topicName>/spur/laser/scan</topicName>
              <frameName>/hokuyo_sensor_link</frameName>
            </plugin>
          </sensor>
        </link>

        <joint name="hokuyo_joint" type="fixed">
          <pose>0 0 0 0 0 0</pose>
          <parent>iris::iris_demo::iris::base_link</parent>
          <child>hokuyo_link</child>
        </joint>


    </model>
    
  </world>
</sdf>
```

---

### Resources 

- http://gazebosim.org/tutorials?tut=ros_gzplugins#Pluginsavailableingazebo_plugins





