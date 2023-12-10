package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
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
        robot.addGamepads(gamepad1, gamepad2);
        SmartGamepad smartGamepad1 = robot.smartGamepad1;
        SmartGamepad smartGamepad2 = robot.smartGamepad2;

        NanoClock clock = NanoClock.system();
        double prevTime = clock.seconds();
        int intakePosition = 0; // 0 = outtake; 1 = intake;
        double intakeStartTime = 0;

        while (!isStopRequested()) {
            telemetry.update();
            robot.update();

            boolean slowMode = gamepad1.left_bumper;
            double joystickRadius = Math.min(1,Math.sqrt(Math.pow(gamepad1.left_stick_y,2) + Math.pow(gamepad1.left_stick_x,2)));
            double factor = robot.mecanumDrive.mapJsRadiusVal(joystickRadius,slowMode);
            double jsX = robot.mecanumDrive.mapJsComponents(-gamepad1.left_stick_x, joystickRadius, slowMode);
            double jsY = robot.mecanumDrive.mapJsComponents(gamepad1.left_stick_y, joystickRadius, slowMode);
            robot.mecanumDrive.setDrivePower(new Pose2d(-jsY, -jsX, -(0.8)*gamepad1.right_stick_x));
            robot.mecanumDrive.setPowerFactor(0.7); //remove with actual robot.


//            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x));

            // INTAKE
            if(smartGamepad1.a_pressed()){
                if(intakePosition == 0 && robot.intake.intakeState == 0) {
                    robot.intake.setIntakeState(1);
                    intakePosition=1;
                } else if (intakePosition == 1 && robot.intake.intakeState == 1){
                    robot.intake.setIntakeState(2);
                    intakePosition=0;
                }
            }
            if(smartGamepad1.b_pressed()){
                robot.intake.toBasePos();
            }
            if(smartGamepad1.x_pressed()){ // test auto output command
                robot.intake.intakeState = 4;
            }
            if(smartGamepad1.right_bumper){// reverses the intake motor for a few seconds :)
                robot.intake.intakeState = 7;

            }


            // Outtake automated
            if(smartGamepad2.a_pressed()){
                robot.intake.setPower(0);
                robot.intake.toBasePos();
                robot.outtake.prepOuttake();
            }

            if (smartGamepad2.right_bumper) {
                robot.outtake.dropPixelPos();
            }

            if (smartGamepad2.x_pressed()) {
                robot.outtake.toIntakePos();
            }

            // Drone launcher
            if(smartGamepad2.left_bumper){
                robot.droneLauncher.release();
            }

                //UG OUTTAKE, single steps
            if(smartGamepad2.left_trigger>0) { robot.outtake.moveDumper(-0.5);}
            if(smartGamepad2.right_trigger>0) {robot.outtake.moveDumper( 0.5);}

            if (smartGamepad2.dpad_right) {
                robot.outtake.moveArm(0.5);
            }
            if (smartGamepad2.dpad_left) {
                robot.outtake.moveArm(-0.5);
            }

            if(smartGamepad2.b_pressed()){
                robot.outtake.toDumpPos();
            }

            if (smartGamepad2.y_pressed()) {
                robot.outtake.toTravelPos();
            }

            if(smartGamepad2.right_stick_button){
                    robot.outtake.prepHang();
            }

            if (smartGamepad2.dpad_up) {
                robot.outtake.lift.adjustLift(1, false);
                telemetry.addLine("dpad up pressed");
                Log.v("PIDLift: gamepad", "dpad up");
            }
            else if (smartGamepad2.dpad_down) {
                robot.outtake.lift.adjustLift(-1, false);
                telemetry.addLine("dpad down pressed");
                Log.v("PIDLift: gamepad", "dpad down");
            } else if (robot.outtake.lift.isLevelReached()){
                robot.outtake.lift.stopMotor();
            }

            telemetry.addData("intake pos", intakePosition);
            //telemetry.addData("intake motor power", robot.intake.getPower());

            telemetry.addData("right servo position: ", robot.outtake.get_RightServoPos());
            telemetry.addData("left servo position: ", robot.outtake.get_LeftServoPos());
            telemetry.addData("dumper servo position: ", robot.outtake.getDumperPos());
            telemetry.addData("slide pos", robot.outtake.getLiftPos());
            telemetry.addData("slide power", robot.outtake.getLiftPower());
            //Log.v("arm", "right servo position: "+ robot.outtake.getRightServoPos());
            double currentTime = clock.seconds();
            telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
            }
        }
    }
