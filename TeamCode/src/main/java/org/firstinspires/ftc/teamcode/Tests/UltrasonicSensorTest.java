package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.RangeSensor;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name= "UltrasonicTest")
public class UltrasonicSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false, false);
        RangeSensor ultrasonic = new RangeSensor(this);
        ElapsedTime op = new ElapsedTime();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("voltage", ultrasonic.getVoltage(true));
            telemetry.addData("distance", ultrasonic.getDistance(true));
            telemetry.addData("voltage", ultrasonic.getVoltage(false));
            telemetry.addData("distance", ultrasonic.getDistance(false));
            telemetry.addData("x", ultrasonic.getLocation()[0]);
            telemetry.addData("y", ultrasonic.getLocation()[1]);
            robot.track();
        }
        stop();
    }
}
