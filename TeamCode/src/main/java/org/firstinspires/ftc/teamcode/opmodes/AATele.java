package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;

@TeleOp
public class AATele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        waitForStart();
        robot.addGamepads(gamepad1,gamepad2);
        SmartGamepad smartGamepad1 = robot.smartGamepad1;
        SmartGamepad smartGamepad2 = robot.smartGamepad2;

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

            if(smartGamepad2.dpad_right){
                robot.outtake.moveArm(0.1);
            } if(smartGamepad2.dpad_left) {
                robot.outtake.moveArm(-0.1);
            }

            if(smartGamepad2.left_trigger>0) { robot.outtake.moveDumper(-0.1);}
            if(smartGamepad2.right_trigger>0) {robot.outtake.moveDumper( 0.1);}

            if (smartGamepad2.x_pressed()) {
                robot.outtake.toIntakePos();
            } if(smartGamepad2.b_pressed()){
                robot.outtake.toDumpPos(2);
            } if(smartGamepad2.right_bumper){
                robot.outtake.dropPixelPos();
            }

            telemetry.addData("right servo position: ", robot.outtake.getRightServoPos());
            telemetry.addData("left servo position: ", robot.outtake.getLeftServoPos());
            telemetry.addData("dumper servo position: ", robot.outtake.getDumperPos());
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());

        }
    }
}
