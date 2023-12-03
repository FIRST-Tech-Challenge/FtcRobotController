package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotVision;
import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;
import org.firstinspires.ftc.teamcode.util.Utilities;

@TeleOp
public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Utilities.getSharedUtility().initialize(this);
        CrabRobot robot = new CrabRobot(this);
        waitForStart();
        robot.addGamepads(gamepad1, gamepad2);
        SmartGamepad smartGamepad1 = robot.smartGamepad1;
        SmartGamepad smartGamepad2 = robot.smartGamepad2;
        RobotVision rvis = new RobotVision();

        NanoClock clock = NanoClock.system();
        double prevTime = clock.seconds();

        int elementPos = 3;

        while (!isStopRequested()) {
            telemetry.update();
            robot.update();
            //EDWARD'S INTAKE
            boolean buttonA = gamepad2.a;
            boolean buttonB = gamepad2.b;

/*
            if(buttonA) {
                robot.intake.setPower(1.0);
            }
            if(buttonB){
                robot.intake.setPower(0);
            }
            //robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x));


                //UG OUTTAKE
            if(smartGamepad2.left_trigger>0) { robot.outtake.moveDumper(-0.1);}
            if(smartGamepad2.right_trigger>0) {robot.outtake.moveDumper( 0.1);}
            if (smartGamepad2.dpad_right) {
                robot.outtake.moveArm(0.1);
            }
            if (smartGamepad2.dpad_left) {
                robot.outtake.moveArm(-0.1);
            }

            if (smartGamepad2.x_pressed()) {
                robot.outtake.toIntakePos();
            }
            if (smartGamepad2.y_pressed()) {
                robot.outtake.toDumpPos();
            }
            if (smartGamepad2.right_bumper) {
                robot.outtake.dropPixelPos();
            }

 */
            if (smartGamepad2.dpad_up_pressed()) {
                robot.dualMotorLift.goToRelativeOffset(2);
                telemetry.addLine("dpad up pressed");
                //Log.v("PIDLift: gamepad", "dpad up");
            }
            else if (smartGamepad2.dpad_down_pressed()) {
                robot.dualMotorLift.goToRelativeOffset(-2);
                telemetry.addLine("dpad down pressed");
                //Log.v("PIDLift: gamepad", "dpad down");
            }

            //gamepad 1 should control intake?
            /*
            if(smartGamepad2.a_pressed()){
                robot.intake.toIntakePos();
            }
            if(smartGamepad2.b_pressed()){
                robot.intake.toOuttakePos();
            }
            if(smartGamepad2.left_bumper){
                robot.intake.toBasePos();
                robot.outtake.prepOuttake();
            }

             */


            telemetry.addData("right servo position: ", robot.outtake.get_RightServoPos());
            telemetry.addData("left servo position: ", robot.outtake.get_LeftServoPos());
            telemetry.addData("dumper servo position: ", robot.outtake.getDumperPos());
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());
            double currentTime = clock.seconds();
            telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
            }
        }
    }
