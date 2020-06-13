# Designing a Multi-rotor Drone 

This document goes through the IQ design process for designing drones for intelligent missions. All of the autonomous drones I have designed have been for research purposes. From which I have noticed that is not a lot of material about designing and building your own custom drone for this purpose. I have tried to aggregate all of the information I have learned over the years into one document. I have also developed a fairly robust design process to help guide new and old drone engineers through their design choices. 

## Overview of the Process
1. Figure out what you want your drone to do 
2. Figure out your sensor package 
3. Design a frame to carry your payload
4. Estimate the weight of the frame and payload
5. Do power thrust analysis and select motors and escs and batteries

## What will my drone do?

This is where you take the mission and decide what you will design to accomplish the goal. Start thinking of how your drone will fly, navigate, take data

## Sensor Compute and Comms Selection 

You should figure out all of the mission specific hardware computers and sensors before you design the vehicle to move your payload.

### Payload Components Most Intelligent Drones Need
- FCC
- Companion Computer
- Camera(s)
- RC receiver 
- Telemetry Radio
- Navigation equipment (GPS, OF, altimeter)
- Avoidance sensors (lidar, sonar, more cameras)

## Designing a Frame

How do we best arrange our sensors, compute and comms to accomplish the mission. Make a CAD to best lock down the spacial arrangement of the components. Use the CAD to get a better weight estimate.

## Picking out motors ESCs and Props 

This part of the process is the most tricky. All of these bits and pieces are coupled, so changing one affects the other. This is not a well defined process and in its current state most hobbyists rely on trial and error, as well as word of mouth. More of an art than a science. I will attempt to make this more of a science based on my experience as well as use some aerospace principals I learned in college. 

It is helpful to think of what our ultimate goal is when designing a drone. We want to carry payload and we want to be able to carry it for as long as possible usually.  While we may think we have constraints on the flight time and payload, ultimately people want to add more sensors and make the vehicle do more things. As the designer to the aircraft, we should try to give the mission designers as much margin as possible. To get the most out of our system we should analyse the efficiency of our system and try to optimize each components. The 2 main design principles I will try to optimize are 
- Aerodynamic/Mechanical Efficiency 
- Electrical Efficiency 

## Aerodynamic Efficiency of a Multi-rotor

The Aerodynamic efficiency of your multi-rotor is affected by a couple different parameters. 
- The propeller disk 
- The pitch of the propeller 
- Blade count 
- The mass of the vehicle

## Propeller Disk 

A multi-rotor works by taking mass(air) and throwing it down so the vehicle can hover or ascend. This is Newton’s third law in a nutshell: for an action, there is an equal and opposite reaction.  To generate more thrust we want to throw as much air as possible down, one way to do this is to increase the propeller disk diameter. When we look at the equation for a circle we notice that the area increases by the square of the radius, so a small increase in the radius will net us a whole heap more of air!

Principal:
- A bigger disk area equals a lot more thrust 

## Propeller Pitch

By the logic of wanting to throw as much air down as we can allows us to increase thrust by throwing the air down faster. This can be accomplished by increasing the pitch of your blade. So great let’s have a large rotor and high pitch! Not so fast. Increasing the pitch of the propeller increases the drag of the rotor, and the added pitch doesn’t transfer the energy from the rotor to the air very well.

Principal:
- Higher pitch equals more thrust
- Higher pitch equals low efficiency
  
## Blade Count

Similar to blade pitch, more blades will give higher thrust, but also increases the drag of the blade as well as the mass. 

Principal:
- More blades equal more thrust
- More blades equal lower efficiency

## Mass of Vehicle

Basically there is a decent amount of math that goes into this, but the big takeaway is this equation.



We notice that the power to hover increases to the power of 1.5, which means adding double the mass requires almost 3 times the power! This is a huge mistake people make: they “I’m only going to add a little bit of weight”, but then their flight time decrease quite a bit. 

Principal:
- Required power increase to the 1.5 power of mass

## Aerodynamic Efficiency Summary

- A bigger disk area equals more thrust by the square of the radius
- Higher pitch equals more thrust
- Higher pitch equals low efficiency
- More blades equal more thrust
- More blades equal low efficiency
- Required power increase to the 1.5 power of mass


## Electrical Efficiency of the Drone
The basic idea behind analyzing the electrical efficiency is to minimize the resistance loss in the circuit. since P=I^2R this means that the power consumed by the resistance in the wires increase with the square of current. This means having a higher voltage system, which in turn reduces the current is a more efficient electrical system.  

## Selecting Motors ECSs and Battery
Now that we understand how to best use the energy stored within our drone we can look at selecting motors and escs. 

The first thing to do is take the weight estimate you calculated from our CAD and parts spread sheet, and use this as the minimum thrust our motors will need at 50% throttle. The best thing to do is take the estimated weight of the done and add a healthy margin, the more you add the better off you will be. This is especially important if this a developmental drone. As you continue to develop your application you will want to add more sensors and actuators to the aircraft, which will kill your flight time quick. As we noted above the power required to hover is to the 1.5 power of your mass! 





