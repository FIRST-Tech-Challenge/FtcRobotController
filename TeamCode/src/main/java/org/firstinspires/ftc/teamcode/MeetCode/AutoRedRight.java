package org.firstinspires.ftc.teamcode.MeetCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoRedRight")
public class AutoRedRight extends LinearOpMode{

    Hardware robot = new Hardware();
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setTargetPosition(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.3);
        robot.wrist.setPosition(1);



        waitForStart();
        while (opModeIsActive()) {
            robot.strafeRightForTime(.6, 2.5);
            sleep(250);
            robot.wrist.setPosition(.2);
            sleep(250);
            robot.claw.setPosition(.4);
            robot.dropper.setPosition(.8);
            sleep(250);
            break;

            /*while (robot.colorSensor.blue() < (robot.colorSensor.red() + robot.colorSensor.green())) {
                robot.setPowerOfAllMotorsTo(.25);
            }
            robot.encoderDrive(-10);
            sleep(250);
            robot.encoderTurn(25.5);
            sleep(250);
            robot.encoderDrive(24);
            robot.squareUp();
            sleep(200);
            robot.cascadeDrive(1000);
            sleep(500);
            robot.arm.setTargetPosition(540);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(.3);
            sleep(250);
            robot.wrist.setPosition(.65);
            robot.claw.setPosition(.4);
            sleep(250);
            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(.3);
            robot.cascadeDrive(-1000);
            sleep(250);
            robot.encoderStrafe(20);
            sleep(250);
            robot.encoderDrive(20);*/
        }
    }
}
