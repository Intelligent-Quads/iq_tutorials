# RC Controller with MAVproxy 

It is help to be able to use an rc controller for manual input when debugging in your simulation. We can do this by using the joystick module in mavproxy. You can connect your rc controller to your pc via a usb dongle.

## Controller Dongles

A List of native dongles are listed below 

https://ardupilot.github.io/MAVProxy/html/modules/joystick.html

I am using a spektrum dx7 controller and elected to by the spektrum wireless receiver listed below 

https://www.amazon.com/Spektrum-Wireless-Simulator-Dongle-WS1000/dp/B06XCP7Z5Y/ref=asc_df_B06XCP7Z5Y/?tag=hyprod-20&linkCode=df0&hvadid=309833041189&hvpos=&hvnetw=g&hvrand=1456726319045727682&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9027263&hvtargid=pla-567767137867&th=1

## Dependencies

make sure all of the below dependencies are installed

```
sudo apt-get install python3-dev python3-opencv python3-wxgtk3.0 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml
``` 

## Run Your Simulation 

```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
```
roslaunch iq_sim runway.launch
```

Once you are running your simulation you can enable your controller by typing the following in your mavproxy terminal
```
module load joystick
``` 

## Custom Joystick Mapping 

