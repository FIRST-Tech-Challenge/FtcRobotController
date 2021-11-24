//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Duck On Y", group = "TeleOp")
@Disabled
public class duckOnYTest extends LinearOpMode{
    // Declaring motors and servos
    DcMotor motorDuck;
    double position = 0;

    //Other Devices
    BNO055IMU imu;
    @Override
    public void runOpMode() {
        //Initialize the motors and servos
        motorDuck = hardwareMap.dcMotor.get("motorDuck");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double position = 0.0;

        //Set power of motors
        while (opModeIsActive()) {

            position = gamepad1.left_stick_y;
            motorDuck.setPower(position);

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