package org.firstinspires.ftc.teamcode.Opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Wrist;
import org.firstinspires.ftc.teamcode.Usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;

import java.util.Locale;


@TeleOp

public class OneDriver extends LinearOpMode {
    public double inchesPerSecond = 66.67;
    double globalStateMachine = 0;
    double oldTime = 0;
    double timeStamp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPosition = new Pose2d(0, 0, 0);
        double slideInches = 0;
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain drive = new Drivetrain(hardwareMap, startPosition);
        Slides slides = new Slides(hardwareMap, drive.getSlidesMotor());
        Arm arm = new Arm(hardwareMap, drive.getArmMotor());
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        stickyGamepad gp = new stickyGamepad(gamepad1);
        waitForStart();
        double frequency = 0;
        double loopTime = 0;

        while (opModeIsActive()) {
            //read gamepads

            if (gp.left_bumper && (globalStateMachine == 11 || globalStateMachine == 12)) {
                globalStateMachine = 11;
            }//advances backwards

            if ((globalStateMachine == 11 || globalStateMachine == 12) && gp.right_bumper) {
                globalStateMachine++;
            } else if (gp.right_bumper) {
                if (globalStateMachine != 9 || globalStateMachine != 6 || globalStateMachine != 3) {
                    globalStateMachine++;
                } //advances forward
            } else if (gp.left_bumper) {
                if (globalStateMachine == 3) {
                    globalStateMachine = 0;
                } else {
                    globalStateMachine--;
                } //advances backwards
            } else if (gp.a) {
                globalStateMachine = 10;
            }//submersible



            if (globalStateMachine < 0) {
                globalStateMachine = 0;
            }//make sure we cant fall out of our state machine



            //state machine
            //if you're reading this PLEASE make this an ENUM or something readable

            if (globalStateMachine == 0) { //default position
                arm.preTake();
                slides.floorIntake();
                wrist.intake();
                claw.open();
            } else if (globalStateMachine == 1) {
                arm.intake();
            } else if (globalStateMachine == 2) {
                claw.close();
                timeStamp = timer.milliseconds();
                globalStateMachine++;
            } else if (globalStateMachine == 3) {
                if (timeStamp + 250 < timer.milliseconds()) {
                    arm.preTake();
                    slides.floorIntake();
                }
            } else if (globalStateMachine == 4) {
                arm.deposit();
                slides.preScore();
            } else if (globalStateMachine == 5) {
                slides.score();
                timeStamp = timer.milliseconds();
                globalStateMachine++;
            } else if (globalStateMachine == 6) {
                if (timeStamp + 400 < timer.milliseconds()) {
                    wrist.deposit();
                }
            } else if (globalStateMachine == 7) {
                claw.open();
            } else if (globalStateMachine == 8) {
                wrist.intake();
                slides.preScore();
                timeStamp = timer.milliseconds();
                globalStateMachine++;
            } else if (globalStateMachine == 9) {
                if (timeStamp + 450 < timer.milliseconds()) {
                    globalStateMachine = 0;
                }
            } else if (globalStateMachine == 10) { // submersible intake
                wrist.straight();
                arm.preSubmerse();
                claw.open();
                globalStateMachine++;
            } else if (globalStateMachine == 11) {

                double increment = inchesPerSecond * loopTime * (gamepad1.left_trigger - gamepad1.right_trigger);
                slideInches += increment;
                if (slideInches > 15.0) slideInches = 15.0;
                slides.setTargetSlidesPosition(slideInches);

            } else if (globalStateMachine == 12) {
                arm.intake();
                wrist.intake();
            } else if (globalStateMachine == 13) {
                claw.close();
            } else if (globalStateMachine == 14) {
                wrist.deposit();
                arm.preSubmerse();
                timeStamp = timer.milliseconds();
                globalStateMachine++;
            } else if (globalStateMachine == 15) {
                if (timeStamp + 350 < timer.milliseconds()) {
                    arm.preTake();
                    slides.floorIntake();
                    timeStamp = timer.milliseconds();
                    globalStateMachine = 16;
                }
            } else if (globalStateMachine == 16) {
                if (timeStamp + 350 < timer.milliseconds()) {
                    wrist.intake();
                    globalStateMachine = 3;
                }
            }




            double newTime = getRuntime();
            loopTime = newTime - oldTime;
            frequency = 1 / loopTime;
            oldTime = newTime;


            drive.update();
            drive.setPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            slides.update();
            arm.update();
            gp.update();


            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", drive.getPose().getX(), drive.getPose().getY(), M.toDegrees(drive.getPose().getHeading()));
            telemetry.addData("Position", data);
            telemetry.addData("Heading Velocity: ", drive.getHeadingVelocity());
            telemetry.addData("Status", drive.getStatus());
            telemetry.addData("Hub loop Time: ", frequency);
            telemetry.addData("slides inches: ", slides.getCurrentSlidesPosition());
            telemetry.addData("arm degrees:", arm.getCurrentArmPosition());
            telemetry.addData("Left Trigger:", gamepad1.left_trigger);
            telemetry.addData("Right Trigger:", gamepad1.right_trigger);

            telemetry.update();
        }
    }
}
