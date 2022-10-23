package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
//  Controls:
// left stick forward and backward
// right stick left and right to strafe
// left stick left and right to turn
// a to move linear slide up
// b to move linear slide down

@TeleOp(name = "Mecanum")
public class Mecanum extends LinearOpMode {

    double frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive, armServo, clawServo, linearslide;
    double driveSpeed = 1;

    @Override
    public void runOpMode() {
        TurtleRobot robot = new TurtleRobot(this);
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                linearslide = (gamepad2.left_stick_y);
            //                     FORWARD                     TURN                       STRAFE
            frontRightDrive = (-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x)*driveSpeed;
            frontLeftDrive  = (-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x)*driveSpeed;
            backRightDrive  = (-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x)*driveSpeed;
            backLeftDrive   = (-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x)*driveSpeed;
            clawServo = (gamepad2.right_stick_x);
            armServo = (gamepad2.right_stick_y);

            robot.rightfrontmotor.setPower(frontRightDrive);
            robot.rightbackmotor.setPower(backRightDrive);
            robot.leftbackmotor.setPower(backLeftDrive);
            robot.leftfrontmotor.setPower(frontLeftDrive);
            robot.ArmServo.setPower(clawServo);
            robot.ClawMotor.setPower(armServo);
            robot.leftslidemotor.setPower(linearslide);
            robot.rightslidemotor.setPower(linearslide);
            if (frontLeftDrive>0 && frontRightDrive>0 && backLeftDrive>0 && backRightDrive>0) {
                telemetry.addLine("Going forward");
            }
            if (frontLeftDrive>0 && frontRightDrive>0 && backLeftDrive<0 && backRightDrive<0 || frontLeftDrive<0 && frontRightDrive<0 && backLeftDrive<0 && backRightDrive<0) {
                telemetry.addLine("Turning");
            }
            if (frontLeftDrive>0 && frontRightDrive<0 && backLeftDrive>0 && backRightDrive<0 || frontLeftDrive<0 && frontRightDrive>0 && backLeftDrive<0 && backRightDrive>0) {
                telemetry.addLine("Strafing");
            }


            telemetry.addLine("motor name               motor speed");
            telemetry.addLine();
            telemetry.addData("Front right drive power = ", frontRightDrive);
            telemetry.addData("Front left drive power  = ", frontLeftDrive);
            telemetry.addData("Back right drive power  = ", backRightDrive);
            telemetry.addData("Back left drive power   = ", backLeftDrive);
            telemetry.update();
            }
        }
    }
}
