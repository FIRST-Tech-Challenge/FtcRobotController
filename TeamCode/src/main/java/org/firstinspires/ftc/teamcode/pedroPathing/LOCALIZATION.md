## Overview
This is the localization system developed for the Pedro Pathing path follower. These localizers use
the pose exponential method of localization. It's basically a way of turning movements from the
robot's coordinate frame to the global coordinate frame. If you're interested in reading more about
it, then check out pages 177 - 183 of [Controls Engineering in the FIRST Robotics Competition](https://file.tavsys.net/control/controls-engineering-in-frc.pdf)
by Tyler Veness. However, the OTOS localizer uses its own onboard system for calculating localization,
which we do not know about.

## Setting Your Localizer
Go to line `70` in the `PoseUpdater` class, and replace the `new ThreeWheelLocalizer(hardwareMap)`
with the localizer that applies to you:
* If you're using drive encoders, put `new DriveEncoderLocalizer(hardwareMap)`
* If you're using two wheel odometry, put `new TwoWheelLocalizer(hardwareMap)`
* If you're using three wheel odometry, put `new ThreeWheelLocalizer(hardwareMap)`, so basically
  don't change it from the default
* If you're using three wheel odometry with the IMU, put `new ThreeWheelIMULocalizer(hardwareMap)`
* If you're using OTOS, put `new OTOSLocalizer(hardwareMap)`
* If you're using Pinpoint, put `new PinpointLocalizer(hardwareMap)`

## Tuning
To start, you'll want to select your localizer of choice. Below, I'll have instructions for the drive
encoder localizer, two tracking wheel localizer, the three tracking wheel localizer, the three
wheel with IMU localizer, and the OTOS localizer offered in Pedro Pathing. Scroll down to the section
that applies to you and follow the directions there.

# Drive Encoders
* First, you'll need all of your drive motors to have encoders attached.
* Then, go to `DriveEncoderLocalizer.java`. The motor names are already set, so you don't have to do
  anything to change the encoder names there.
* Then, reverse the direction of any encoders so that all encoders tick up when the robot is moving forward.
* Now, you'll have to tune the multipliers. These convert your measurements from encoder ticks into 
  inches or radians, essentially scaling your localizer so that your numbers are accurate to the real
  world.
* First, start with the `Turn Localizer Tuner`. You'll want to position your robot to be facing
  in a direction you can easily find again, like lining up an edge of the robot against a field tile edge.
  By default, you should spin the robot for one rotation going counterclockwise. Once you've spun
  exactly that one rotation, or whatever you set that value to, then the turn multiplier will be shown
  as the second number shown. The first number is how far the robot thinks you've spun, and the second
  number is the multiplier you need to have to scale your current readings to your goal of one rotation,
  or the custom set angle. Feel free to run a few more tests and average the results. Once you have
  this multiplier, then replace `TURN_TICKS_TO_RADIANS` in the localizer with your multiplier. Make sure
  you replace the number, not add on or multiply it to the previous number. The tuner takes into
  account your current multiplier.
* Next, go on to `Forward Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches forward. Once you've pushed that far, or whatever
  you set that value to, then the forward multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `FORWARD_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Finally, go to `Lateral Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches to the right. Once you've pushed that far, or whatever
  you set that value to, then the lateral multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `STRAFE_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Once you're done with all this, your localizer should be tuned. To test it out, you can go to
 `Localization Test` and push around or drive around your robot. Go to [FTC Dashboard](http://192.168.43.1:8080/dash)
 and on the top right, switch the drop down from the default view to the field view. Then, on the bottom
 left corner, you should see a field and the robot being drawn on the field. You can then move your
 robot around and see if the movements look accurate on FTC Dashboard. If they don't, then you'll
 want to re-run some of the previous steps. Otherwise, congrats on tuning your localizer!

# Two Wheel Localizer
* First, you'll need a Control Hub with a working IMU, and two odometry wheels connected to motor
  encoder ports on a hub.
* Then, go to `TwoWheelLocalizer.java`. First, in the constructor, enter in the positions of your
 tracking wheels relative to the center of the wheels of the robot. The positions are in inches, so
 convert measurements accordingly. Use the comment above the class declaration to help you with the
 coordinates.
* Next, go to where it tells you to replace the current statements with your encoder ports in the constructor.
  Replace the `deviceName` parameter with the name of the port that the encoder is connected to. The
 variable names correspond to which tracking wheel should be connected.
* After that, go to the instantiation of the IMU and change the orientation of the IMU to match that
  of your robot's.
* Then, reverse the direction of any encoders so that the forward encoder ticks up when the robot
 is moving forward and the strafe encoder ticks up when the robot moves right.
* Now, you'll have to tune the multipliers. These convert your measurements from encoder ticks into
  inches or radians, essentially scaling your localizer so that your numbers are accurate to the real
  world.
* You actually won't need the turning tuner for this one, since the IMU in the Control Hub will take
 care of the heading readings.
* First, start with the `Forward Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches forward. Once you've pushed that far, or whatever
  you set that value to, then the forward multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `FORWARD_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Finally, go to `Lateral Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches to the right. Once you've pushed that far, or whatever
  you set that value to, then the lateral multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `STRAFE_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Once you're done with all this, your localizer should be tuned. To test it out, you can go to
 `Localization Test` and push around or drive around your robot. Go to [FTC Dashboard](http://192.168.43.1:8080/dash)
 and on the top right, switch the drop down from the default view to the field view. Then, on the bottom
 left corner, you should see a field and the robot being drawn on the field. You can then move your
 robot around and see if the movements look accurate on FTC Dashboard. If they don't, then you'll
 want to re-run some of the previous steps. Otherwise, congrats on tuning your localizer!

# Three Wheel Localizer
* First, you'll need three odometry wheels connected to motor encoder ports on a hub.
* Then, go to `ThreeWheelLocalizer.java`. First, in the constructor, enter in the positions of your
  tracking wheels relative to the center of the wheels of the robot. The positions are in inches, so
  convert measurements accordingly. Use the comment above the class declaration to help you with the
  coordinates.
* Next, go to where it tells you to replace the current statements with your encoder ports in the constructor.
  Replace the `deviceName` parameter with the name of the port that the encoder is connected to. The
  variable names correspond to which tracking wheel should be connected.
* Then, reverse the direction of any encoders so that the forward encoders tick up when the robot
  is moving forward and the strafe encoder ticks up when the robot moves right.
* First, start with the `Turn Localizer Tuner`. You'll want to position your robot to be facing
  in a direction you can easily find again, like lining up an edge of the robot against a field tile edge.
  By default, you should spin the robot for one rotation going counterclockwise. Once you've spun
  exactly that one rotation, or whatever you set that value to, then the turn multiplier will be shown
  as the second number shown. The first number is how far the robot thinks you've spun, and the second
  number is the multiplier you need to have to scale your current readings to your goal of one rotation,
  or the custom set angle. Feel free to run a few more tests and average the results. Once you have
  this multiplier, then replace `TURN_TICKS_TO_RADIANS` in the localizer with your multiplier. Make sure
  you replace the number, not add on or multiply it to the previous number. The tuner takes into
  account your current multiplier.
* Next, go on to `Forward Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches forward. Once you've pushed that far, or whatever
  you set that value to, then the forward multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `FORWARD_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Finally, go to `Lateral Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches to the right. Once you've pushed that far, or whatever
  you set that value to, then the lateral multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `STRAFE_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Once you're done with all this, your localizer should be tuned. To test it out, you can go to
  `Localization Test` and push around or drive around your robot. Go to [FTC Dashboard](http://192.168.43.1:8080/dash)
  and on the top right, switch the drop down from the default view to the field view. Then, on the bottom
  left corner, you should see a field and the robot being drawn on the field. You can then move your
  robot around and see if the movements look accurate on FTC Dashboard. If they don't, then you'll
  want to re-run some of the previous steps. Otherwise, congrats on tuning your localizer!

# Three Wheel Localizer with IMU
* First, you'll need three odometry wheels connected to motor encoder ports on a hub.
* Then, go to `ThreeWheelIMULocalizer.java`. First, in the constructor, enter in the positions of your
  tracking wheels relative to the center of the wheels of the robot. The positions are in inches, so
  convert measurements accordingly. Use the comment above the class declaration to help you with the
  coordinates.
* Next, go to where it tells you to replace the current statements with your encoder ports in the constructor.
  Replace the `deviceName` parameter with the name of the port that the encoder is connected to. The
  variable names correspond to which tracking wheel should be connected.
* After that, go to the instantiation of the IMU and change the orientation of the IMU to match that
  of your robot's.
* Then, reverse the direction of any encoders so that the forward encoders tick up when the robot
  is moving forward and the strafe encoder ticks up when the robot moves right.
* Although heading localization is done mostly through the IMU, the tracking wheels are still used for
  small angle adjustments for better stability. So, you will still need to tune your turning multiplier.
* First, start with the `Turn Localizer Tuner`. Before doing any tuning, go to FTC Dashboard and find
  the `ThreeWheelIMULocalizer` dropdown and deselect `useIMU`. You'll want to position your robot to be facing
  in a direction you can easily find again, like lining up an edge of the robot against a field tile edge.
  By default, you should spin the robot for one rotation going counterclockwise. Once you've spun
  exactly that one rotation, or whatever you set that value to, then the turn multiplier will be shown
  as the second number shown. The first number is how far the robot thinks you've spun, and the second
  number is the multiplier you need to have to scale your current readings to your goal of one rotation,
  or the custom set angle. Feel free to run a few more tests and average the results. Once you have
  this multiplier, then replace `TURN_TICKS_TO_RADIANS` in the localizer with your multiplier. Make sure
  you replace the number, not add on or multiply it to the previous number. The tuner takes into
  account your current multiplier.
* Next, go on to `Forward Localizer Tuner`. You should re-enable `useIMU` at this time. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches forward. Once you've pushed that far, or whatever
  you set that value to, then the forward multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `FORWARD_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Finally, go to `Lateral Localizer Tuner`. `useIMU` should be enabled for this step. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches to the right. Once you've pushed that far, or whatever
  you set that value to, then the lateral multiplier will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the multiplier you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this multiplier, then
  replace `STRAFE_TICKS_TO_INCHES` in the localizer with your multiplier. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current multiplier.
* Once you're done with all this, your localizer should be tuned. Make sure that `useIMU` is turned back on. To test it out, you can go to
  `Localization Test` and push around or drive around your robot. Go to [FTC Dashboard](http://192.168.43.1:8080/dash)
  and on the top right, switch the drop down from the default view to the field view. Then, on the bottom
  left corner, you should see a field and the robot being drawn on the field. You can then move your
  robot around and see if the movements look accurate on FTC Dashboard. If they don't, then you'll
  want to re-run some of the previous steps. Otherwise, congrats on tuning your localizer!

# OTOS Localizer
* First, you'll need the OTOS connected to an I2C port on a hub. Make sure the film on the sensor is removed.
* Then, go to `OTOSLocalizer.java`. First, in the constructor, go to where it tells you to replace 
  the current statement with your OTOS port in the constructor. Replace the `deviceName` parameter
  with the name of the port that the OTOS is connected to.
* Next, enter in the position of your OTOS relative to the center of the wheels of the robot. The
  positions are in inches, so convert measurements accordingly. Use the comment above the class
  declaration as well as to help you with the coordinates.
* First, start with the `Turn Localizer Tuner`. You'll want to position your robot to be facing
  in a direction you can easily find again, like lining up an edge of the robot against a field tile edge.
  By default, you should spin the robot for one rotation going counterclockwise. Once you've spun
  exactly that one rotation, or whatever you set that value to, then the angular scalar will be shown
  as the second number shown. The first number is how far the robot thinks you've spun, and the second
  number is the scalar you need to have to scale your current readings to your goal of one rotation,
  or the custom set angle. Feel free to run a few more tests and average the results. Once you have
  this scalar, then replace the angular scalar on line `78` in the localizer with your scalar.
  Make sure you replace the number, not add on or multiply it to the previous number. The tuner takes into
  account your current angular scalar.
* For this next step, since OTOS only has one linear scalar, you can run either the forward or lateral
  localizer tuner and the result should be the same. So, you choose which one you want to run.
* Option 1: go on to `Forward Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches forward. Once you've pushed that far, or whatever
  you set that value to, then the linear scalar will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the scalar you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this scalar, then
  replace the linear scalar on line `77` in the localizer with your scalar. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current scalar.
* Option 2: go to `Lateral Localizer Tuner`. You'll want to position a ruler alongside your robot.
  By default, you'll want to push the robot 30 inches to the right. Once you've pushed that far, or whatever
  you set that value to, then the linear scalar will be shown as the second number shown. The
  first number is how far the robot thinks you've gone, and the second number is the scalar you
  need to have to scale your current readings to your goal of 30 inches, or the custom set distance.
  Feel free to run a few more tests and average the results. Once you have this scalar, then
  replace the linear scalar on line `77` in the localizer with your scalar. Make sure you replace the number,
  not add on or multiply it to the previous number. The tuner takes into account your current scalar.
* Once you're done with all this, your localizer should be tuned. To test it out, you can go to
  `Localization Test` and push around or drive around your robot. Go to [FTC Dashboard](http://192.168.43.1:8080/dash)
  and on the top right, switch the drop down from the default view to the field view. Then, on the bottom
  left corner, you should see a field and the robot being drawn on the field. You can then move your
  robot around and see if the movements look accurate on FTC Dashboard. If they don't, then you'll
  want to re-run some of the previous steps. Otherwise, congrats on tuning your localizer!

## Pinpoint Localizer
* First you will need to plug in the pinpoint to the i2c port. Ensure that the dead wheel encoder wires are
  plugged into the proper ports on the pinpoint to ensure no error within the tuning steps.
* Then, go to the `PinpointLocalier.java` file and go to where it tells you to replace
  the current statement with your pinpoint port in the constructor. Replace the `deviceName` parameter
  with the name of the port that the pinpoint is connected to.
* Next, follow the instructions left by the TODO: comment and enter in the odometry measurements either in
  mms or inches (We have the conversion rates listed).
* First, to ensure that your pinpoint is properly connected, please run the `SensorGoBildaPinpointExample.java`
  file left in the `tuning` folder located within `localization`.
* Once completed, the localizer should be properly tuned. To test it out, you can go to
  `Localization Test` and push around or drive around your robot. Go to [FTC Dashboard](http://192.168.43.1:8080/dash)
  and on the top right, switch the drop down from the default view to the field view. Then, on the bottom
  left corner, you should see a field and the robot being drawn on the field. You can then move your
  robot around and see if the movements look accurate on FTC Dashboard. If they don't, then you'll
  want to re-run some of the previous steps. Otherwise, congrats on tuning your localizer!

## Using Road Runner's Localizer
Of course, many teams have experience using Road Runner in the past and so have localizers from Road
Runner that are tuned. There is an adapter for the Road Runner three wheel localizer to the Pedro
Pathing localization system in Pedro Pathing, but it is commented out by default to reduce the number
of imports in gradle.

To re-enable it, go to `RoadRunnerEncoder.java`, `RoadRunnerThreeWheelLocalizer.java`, and `RRToPedroThreeWheelLocalizer.java` 
and hit `ctrl` + `a` to select everything within the files. Then, press `ctrl` + `/` to uncomment the code.

Afterwards, go to `build.gradle` file under the `teamcode` folder and add the following dependencies:
```
implementation 'org.apache.commons:commons-math3:3.6.1'
implementation 'com.acmerobotics.com.roadrunner:core:0.5.6'
```

After that, you should be good to go. If you want to use a different localizer from Road Runner, then
you can adapt it in the same process that's used for the Road Runner three wheel localizer.