package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.TurtleRobotTeleOp;
//  Controls:
// left stick forward and backward
// right stick left and right to strafe
// left stick left and right to turn
// a to move linear slide up
// b to move linear slide down

@TeleOp(name = "Mecanum")
public class Mecanum extends LinearOpMode {

    double frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive, armServo, clawServo;
    double driveSpeed = 0.8;

    @Override
    public void runOpMode() {
        TurtleRobotTeleOp robot = new TurtleRobotTeleOp(this);
        robot.init(hardwareMap);
        robot.ArmServo.setPower(0);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
            robot.leftslidemotor.setPower(gamepad2.left_stick_y*0.7);
            robot.rightslidemotor.setPower(gamepad2.left_stick_y*0.7);
            //                     FORWARD                     TURN                       STRAFE
            frontRightDrive = (-gamepad1.left_stick_y - (gamepad1.right_stick_x*0.5) - gamepad1.left_stick_x)*driveSpeed;
            frontLeftDrive  = (-gamepad1.left_stick_y + (gamepad1.right_stick_x*0.5) + gamepad1.left_stick_x)*driveSpeed;
            backRightDrive  = (-gamepad1.left_stick_y - (gamepad1.right_stick_x*0.5) + gamepad1.left_stick_x)*driveSpeed;
            backLeftDrive   = (-gamepad1.left_stick_y + (gamepad1.right_stick_x*0.5) - gamepad1.left_stick_x)*driveSpeed;
            robot.ArmServo.setPower(gamepad2.right_stick_y);
            robot.rightfrontmotor.setPower(frontRightDrive);
            robot.rightbackmotor.setPower(backRightDrive);
            robot.leftbackmotor.setPower(backLeftDrive);
            robot.leftfrontmotor.setPower(frontLeftDrive);

            if (frontLeftDrive>0 && frontRightDrive>0 && backLeftDrive>0 && backRightDrive>0) {
                telemetry.addLine("Going forward");
            }
            if (frontLeftDrive>0 && frontRightDrive>0 && backLeftDrive<0 && backRightDrive<0 || frontLeftDrive<0 && frontRightDrive<0 && backLeftDrive<0 && backRightDrive<0) {
                telemetry.addLine("Turning");
            }
            if (frontLeftDrive>0 && frontRightDrive<0 && backLeftDrive>0 && backRightDrive<0 || frontLeftDrive<0 && frontRightDrive>0 && backLeftDrive<0 && backRightDrive>0) {
                telemetry.addLine("Strafing");
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x*0.5
                    )
            );
            drive.update();
//
            robot.leftslidemotor.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            robot.rightslidemotor.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            while (gamepad2.right_bumper) {
                robot.ArmServo.setPower(1);
            }
            while (gamepad2.left_bumper) {
                robot.ArmServo.setPower(-1);
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
