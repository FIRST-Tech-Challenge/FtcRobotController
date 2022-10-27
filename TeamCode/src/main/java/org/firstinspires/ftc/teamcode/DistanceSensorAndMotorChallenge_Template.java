package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Assignment:
 * Have the motor's power set by the distance sensor. The farther away from an object the
 * distance sensor is, the faster the motor runs. Similarly, the closer an object is to
 * the sensor, the slower the motor runs. As per the demonstration I gave in class.
 *
 * Resources that might be helpful:
 * RunAMotorVariablePower.java
 * SensorREV2mDistance.java
 * The demo I gave (if you want to see it again ask me :) )
 *
 * Hints
 * Check out the Range class...there might be a useful method there
 * Use meters as the units
 * The distance sensor goes from 0.0 to 2.0 meters, and we want the motor to run from 0.0 to 1.0
 */

@Autonomous
@Disabled
public class DistanceSensorAndMotorChallenge_Template extends LinearOpMode{
    @Override
    public void runOpMode(){

    }
}
