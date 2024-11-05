package org.firstinspires.ftc.teamcode.Opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Hang;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Wrist;
import org.firstinspires.ftc.teamcode.Usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;

import java.util.Locale;


@TeleOp

public class OneDriver extends LinearOpMode {
    public double inchesPerSecond = 66.67;
    double oldTime = 0;
    double timeStamp = 0;
    enum GlobalStateMachine {
        DEFAULT, INTAKE_READY, CLOSE_INTAKE, FINISHED_INTAKE, READY_DEPOSIT, SLIDES_BEGIN_SCORE,
        WRIST_SCORE, FINISH_SCORE, POST_SCORE, RETURN_TO_DEFAULT,
        BEGIN_SUBMERSIBLE, SUBMERSIBLE_SLIDER, SUBMERSIBLE_INTAKE_OPEN, SUBMERSIBLE_INTAKE_CLOSE,
        SUBMERSIBLE_FINISH_1, SUBMERSIBLE_FINISH_2, RETURN_TO_MAIN
    }

    GlobalStateMachine globalStateMachine = GlobalStateMachine.DEFAULT;
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
        Hang hang = new Hang(hardwareMap);
        stickyGamepad gp = new stickyGamepad(gamepad1);
        waitForStart();
        double frequency = 0;
        double loopTime = 0;


        while (opModeIsActive()) {
            //read gamepads

            if (gp.left_bumper && (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_SLIDER || globalStateMachine == GlobalStateMachine.SUBMERSIBLE_INTAKE_OPEN)) {
                globalStateMachine = GlobalStateMachine.SUBMERSIBLE_SLIDER;
            }//advances backwards
            if (gp.right_bumper) {
                if(globalStateMachine == GlobalStateMachine.DEFAULT) { // 0 -> 1
                    globalStateMachine = GlobalStateMachine.INTAKE_READY;
                } else if(globalStateMachine == GlobalStateMachine.INTAKE_READY) { // 1 -> 2
                    globalStateMachine = GlobalStateMachine.CLOSE_INTAKE;
                } else if (globalStateMachine == GlobalStateMachine.FINISHED_INTAKE && timeStamp + 250 < timer.milliseconds()) { // 3->4
                    globalStateMachine = GlobalStateMachine.READY_DEPOSIT;
                } else if (globalStateMachine == GlobalStateMachine.READY_DEPOSIT) { // 4 -> 5
                    globalStateMachine = GlobalStateMachine.SLIDES_BEGIN_SCORE;
                } else if (globalStateMachine == GlobalStateMachine.WRIST_SCORE && timeStamp + 400 < timer.milliseconds()) { // 6->7
                    globalStateMachine = GlobalStateMachine.FINISH_SCORE;
                } else if (globalStateMachine == GlobalStateMachine.FINISH_SCORE) { //7->8
                    globalStateMachine = GlobalStateMachine.POST_SCORE;
                }
                else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_SLIDER) { // 11 -> 12
                    globalStateMachine = GlobalStateMachine.SUBMERSIBLE_INTAKE_OPEN;
                } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_INTAKE_OPEN) { // 12 -> 13
                    globalStateMachine = GlobalStateMachine.SUBMERSIBLE_INTAKE_CLOSE;
                } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_INTAKE_CLOSE) {
                    globalStateMachine = GlobalStateMachine.SUBMERSIBLE_FINISH_1;
                }
            }
            else if (gp.left_bumper) {//advances backwards
                if (globalStateMachine == GlobalStateMachine.INTAKE_READY || globalStateMachine == GlobalStateMachine.FINISHED_INTAKE) {
                    globalStateMachine = GlobalStateMachine.DEFAULT;
                } else if (globalStateMachine == GlobalStateMachine.READY_DEPOSIT){
                    globalStateMachine = GlobalStateMachine.FINISHED_INTAKE;
                } else if (globalStateMachine == GlobalStateMachine.WRIST_SCORE) {
                    globalStateMachine = GlobalStateMachine.READY_DEPOSIT;
                }  else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_INTAKE_CLOSE) {
                    globalStateMachine = GlobalStateMachine.SUBMERSIBLE_SLIDER;
                }
            } else if (gp.a && (globalStateMachine == GlobalStateMachine.DEFAULT || globalStateMachine == GlobalStateMachine.INTAKE_READY)  ) { // safeguard for submersible
                globalStateMachine = GlobalStateMachine.BEGIN_SUBMERSIBLE;
            } else if (gp.a && (globalStateMachine == GlobalStateMachine.BEGIN_SUBMERSIBLE || globalStateMachine == GlobalStateMachine.SUBMERSIBLE_SLIDER)) { // switch back from submersible
                globalStateMachine = GlobalStateMachine.DEFAULT;
            }



            //state machine
            if (globalStateMachine == GlobalStateMachine.DEFAULT) { // DEFAULT
                arm.preTake();
                slides.floorIntake();
                wrist.intake();
                claw.open();
            } else if (globalStateMachine == GlobalStateMachine.INTAKE_READY) { //INTAKE READY
                arm.intake();
            } else if (globalStateMachine == GlobalStateMachine.CLOSE_INTAKE) { //CLOSE INTAKE
                claw.close();
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.FINISHED_INTAKE;
            } else if (globalStateMachine == GlobalStateMachine.FINISHED_INTAKE) { // FINISHED INTAKE
                if (timeStamp + 250 < timer.milliseconds()) {
                    arm.preTake();
                    slides.floorIntake();
                }
            } else if (globalStateMachine == GlobalStateMachine.READY_DEPOSIT) { // READY DEPOSIT
                arm.deposit();
                slides.preScore();
                wrist.intake();
            } else if (globalStateMachine == GlobalStateMachine.SLIDES_BEGIN_SCORE) { // SLIDES BEGIN SCORE
                slides.score();
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.WRIST_SCORE;
            } else if (globalStateMachine == GlobalStateMachine.WRIST_SCORE) { // WRIST SCORE
                if (timeStamp + 400 < timer.milliseconds()) {
                    wrist.deposit();
                }
            } else if (globalStateMachine == GlobalStateMachine.FINISH_SCORE) { // FINISH SCORE
                claw.open();
            } else if (globalStateMachine == GlobalStateMachine.POST_SCORE) { // POST SCORE
                wrist.intake();
                slides.preScore();
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.RETURN_TO_DEFAULT;
            } else if (globalStateMachine == GlobalStateMachine.RETURN_TO_DEFAULT) { // RETURN TO DEFAULT
                if (timeStamp + 450 < timer.milliseconds()) {
                    globalStateMachine = GlobalStateMachine.DEFAULT;
                }
            } else if (globalStateMachine == GlobalStateMachine.BEGIN_SUBMERSIBLE) { // BEGIN SUBMERSIBLE
                wrist.intake();
                arm.preSubmerse();
                claw.open();
                slideInches = 0;
                globalStateMachine = GlobalStateMachine.SUBMERSIBLE_SLIDER;
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_SLIDER) { // SUBMERSIBLE SLIDER
                wrist.intake();
                arm.preSubmerse();
                claw.open();
                double increment = inchesPerSecond * loopTime * (gamepad1.left_trigger - gamepad1.right_trigger);
                slideInches += increment;
                if (slideInches > 15.0) slideInches = 15.0;
                slides.setTargetSlidesPosition(slideInches);

            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_INTAKE_OPEN) { // SUBMERSIBLE INTAKE OPEN
                claw.open();
                arm.intake();
                wrist.intake();
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_INTAKE_CLOSE) { // SUBMERSIBLE INTAKE CLOSE
                claw.close();
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_FINISH_1) { // SUBMERSIBLE FINISH 1
                wrist.deposit();
                arm.preSubmerse();
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.SUBMERSIBLE_FINISH_2;
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_FINISH_2) { // SUBMERSIBLE FINISH 2
                if (timeStamp + 350 < timer.milliseconds()) {
                    arm.preTake();
                    slides.floorIntake();
                    timeStamp = timer.milliseconds();
                    globalStateMachine = GlobalStateMachine.RETURN_TO_MAIN;
                }
            } else if (globalStateMachine == GlobalStateMachine.RETURN_TO_MAIN) { // RETURN TO MAIN STATE
                if (timeStamp + 350 < timer.milliseconds()) {
                    wrist.intake();
                    globalStateMachine = GlobalStateMachine.FINISHED_INTAKE;
                }
            }
            if (gamepad1.dpad_down){
                hang.move(-1);
            } else if (gamepad1.dpad_up){
                hang.move(1);
            }
            else{
                hang.move(0);
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
