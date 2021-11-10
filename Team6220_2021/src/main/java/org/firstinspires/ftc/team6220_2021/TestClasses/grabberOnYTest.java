//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GrabberOnY", group = "TeleOp")
public class grabberOnYTest extends LinearOpMode{
    // Declaring motors and servos
    Servo servoGrabber;
    double position = 0;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        //Initialize the motors and servos
        servoGrabber = hardwareMap.servo.get("servoGrabber");

        //Set position of servos
        servoGrabber.setPosition(0.3);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double position = 0.0;

        //Set power of motors
        while (opModeIsActive()) {

            position = gamepad1.left_stick_y;
            servoGrabber.setPosition(position);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Position", "Position (%.2f)", position);
            telemetry.update();

        }
    }
    public void pauseMillis(double time) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            idle();
        }
    }
}