package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="DavidDistanceSensor", group = "Sensor")
public class SensorDistanceDavid extends LinearOpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;
    @Override public void runOpMode() {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("cm","%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
