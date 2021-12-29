/*
Max Velocity Test for PIDF Tuning
https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit
Get maxVelocity by running this OpMode

F = 32767 / maxVelocity
P = 0.1 * F
I = 0.1 * P
D = 0
positionP = 5.0

 */
package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FrenzyHardwareMap;

@TeleOp(name="Drive Test", group="Utilities")
public class DriveTest extends LinearOpMode {
    FrenzyHardwareMap robot = new FrenzyHardwareMap();

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        //init loop
        while (! isStarted()) {
            telemetry.addData("Drive Test", "This test is best run on top of a box to test each wheel");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;
        //Code to test if the motors are functioning as intended. Leave commented out if not testing.


        timer.reset();
        while (timer.time() < 5000 && opModeIsActive()){

            robot.motorFrontLeft.setPower(0.5);
            robot.motorBackLeft.setPower(0.5);
            robot.motorBackRight.setPower(0.5);
            robot.motorFrontRight.setPower(0.5);

            telemetry.addData("encder frontLeft", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("encoder frontRight", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("encoder backRight", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("encoder backLeft", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        robot.motorFrontLeft.setPower(0.0);
        robot.motorBackLeft.setPower(0.0);
        robot.motorBackRight.setPower(0.0);
        robot.motorFrontRight.setPower(0.0);

        sleep(1000);

       robot.motorFrontLeft.setPower(0.5);
       telemetry.addData("motor", "front left");
       telemetry.update();
       sleep(5000);
       robot.motorFrontLeft.setPower(0);

       robot.motorFrontRight.setPower(0.5);
       telemetry.addData("motor", "front right");
       telemetry.update();
       sleep(5000);
        robot.motorFrontRight.setPower(0);

        robot.motorBackRight.setPower(0.5);
        telemetry.addData("motor","back right");
        telemetry.update();
        sleep(5000);
        robot.motorBackRight.setPower(0);

        robot.motorBackLeft.setPower(0.5);
        telemetry.addData("motor","back left");
        telemetry.update();
        sleep(5000);
        robot.motorBackLeft.setPower(0);

        robot.motorFrontLeft.setPower(0.5);
        robot.motorFrontRight.setPower(0.5);
        telemetry.addData("motor","front Right and Left");
        telemetry.update();
        sleep(5000);
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);

        robot.motorBackLeft.setPower(0.5);
        robot.motorBackRight.setPower(0.5);
        telemetry.addData("motor","back Right and Left");
        telemetry.update();
        sleep(5000);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);

    }
}
