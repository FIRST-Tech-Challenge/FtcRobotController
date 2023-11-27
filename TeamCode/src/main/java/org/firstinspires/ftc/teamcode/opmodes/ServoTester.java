package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

@TeleOp
public class ServoTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        waitForStart();

        while (!isStopRequested()) {
            //EDWARD'S INTAKE
            boolean buttonA = gamepad2.a;
            boolean buttonB = gamepad2.b;
            //telemetry.addData("left_stick_y:", gamepad1.left_stick_y);
            telemetry.update();
            robot.update();

            /*
            if(buttonA) {
                robot.intake.setPower(1.0);
            }
            if(buttonB){
                robot.intake.setPower(0);
            }
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x));
            */

            //UG OUTTAKE
            boolean dpadUp = gamepad1.dpad_up;//move both +
            boolean dpadDown = gamepad1.dpad_down;//move both arms -
            boolean dpadRight = gamepad1.dpad_right;//adjust right arm +
            boolean dpadLeft = gamepad1.dpad_left;//adjust right arm-
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;

            if(dpadUp){
                robot.outtake.moveArm(0.1);
            } if(dpadDown) {
                robot.outtake.moveArm(-0.1);
            }
            if(dpadRight){
                robot.intake.moveArm(0.1);
            } else if (dpadLeft){
                robot.intake.moveArm(-0.1);
            }
            if(gamepad1.left_bumper){
                robot.intake.setPower(1);
            } else {
                robot.intake.setPower(0);
            }
            /*
            if(dpadRight){
                robot.outtake.moveL(0.1);
            } if(dpadLeft) {
                robot.outtake.moveL(-0.1);
            }
            */
            if (gamepad1.x) {
                robot.outtake.toIntakePos();
                robot.intake.toIntakePos();
            } if(gamepad1.b) {
                robot.outtake.toDumpPos();
                robot.intake.toOuttakePos();
            } if(gamepad1.y) {
                robot.outtake.toTravelPos();
            }

            if (gamepad1.a) {
                robot.outtake.resetArmPos();
            }

            if(gamepad1.right_bumper){
                robot.outtake.dropPixelPos();
            }

            if(leftTrigger>0) { robot.outtake.moveDumper(-0.1);}
            if(rightTrigger>0) {robot.outtake.moveDumper( 0.1);}


            telemetry.addData("right servo position: ", robot.outtake.get_RightServoPos());
            telemetry.addData("left servo position: ", robot.outtake.get_LeftServoPos());
            telemetry.addData("dumper servo position: ", robot.outtake.getDumperPos());
            telemetry.addData("intake left arm position: ", robot.intake.getLeftServoPos());
            telemetry.addData("intake right arm position: ", robot.intake.getRightServoPos());
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());

        }
    }
}
