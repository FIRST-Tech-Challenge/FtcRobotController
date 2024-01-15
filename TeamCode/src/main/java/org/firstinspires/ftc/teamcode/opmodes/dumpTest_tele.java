package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;

@TeleOp
public class dumpTest_tele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        waitForStart();
        robot.addGamepads(gamepad1, gamepad2);
        SmartGamepad smartGamepad1 = robot.smartGamepad1;
        SmartGamepad smartGamepad2 = robot.smartGamepad2;
        RobotDistanceSensor distanceSensor = robot.ds;
        double dumperPos = 0;
        int slideHt = 0;

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

            // do not move
            if(smartGamepad1.right_bumper == false && robot.intake.intakeState == 21){// reverses the intake motor for a few seconds :)
                robot.intake.intakeState = 0;
            }
            if(smartGamepad1.right_bumper){// reverses the intake motor for a few seconds :)
                robot.intake.intakeState = 21;
            }
            if(smartGamepad1.right_trigger < 0.5 && robot.intake.intakeState == 22){// reverses the intake motor for a few seconds :)
                robot.intake.intakeState = 0;
            }
            if(smartGamepad1.right_trigger >= 0.5){// reverses the intake motor for a few seconds :)
                robot.intake.intakeState = 22;
            }

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
                robot.intake.intakeState = 11;
            }




            // Outtake automated
            if(smartGamepad2.a_pressed()){
                telemetry.addData("DumperPos ",dumperPos);
                robot.outtake.setDumpServoPos(dumperPos);
                dumperPos += 0.01;
            }

            if(smartGamepad2.b_pressed()){
                telemetry.addData("DumperPos ",dumperPos);
                robot.outtake.setDumpServoPos(dumperPos);
                dumperPos -= 0.01;
            }
            //Slide
            if(smartGamepad2.dpad_up_pressed()){
                telemetry.addData("Slide Height ", slideHt);
                robot.outtake.lift.goToHtInches(slideHt);
                slideHt += 1;
            }
            if(smartGamepad2.dpad_down_pressed()){
                telemetry.addData("Slide Height ", slideHt);
                robot.outtake.lift.goToHtInches(slideHt);
                slideHt -= 1;
            }
            //Arms
            if(smartGamepad2.right_bumper){
                robot.outtake.armToBackdropPos();
            }
            if(smartGamepad2.left_bumper){
                robot.outtake.armToTravelPosAuto();
            }


            telemetry.addData("DistR: ",distanceSensor.distanceRight());
            telemetry.addData("DistL: ",distanceSensor.distanceLeft());
            double currentTime = clock.seconds();
            //telemetry.addData("Update time: ", currentTime - prevTime);
            prevTime = currentTime;
            }
        }
    }
