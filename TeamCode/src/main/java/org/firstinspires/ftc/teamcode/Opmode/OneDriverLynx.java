package org.firstinspires.ftc.teamcode.Opmode;

import android.provider.Settings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Turret;
import org.firstinspires.ftc.teamcode.RoboActions;
import org.firstinspires.ftc.teamcode.Usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;

import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;


import java.util.Locale;


@TeleOp(name = "! Teleop")
@Config
public class OneDriverLynx extends LinearOpMode {
    public double inchesPerSecond = 66.67;
    double turretPos = 0.5;
    double oldTime = 0;
    double timeStamp = 0;
    public static double turretSpeed = 1.5;
    enum GlobalStateMachine {
        DEFAULT, INTAKE_READY, CLOSE_INTAKE, FINISHED_INTAKE, READY_DEPOSIT, SLIDES_BEGIN_SCORE,
        WRIST_SCORE, FINISH_SCORE, POST_SCORE, RETURN_TO_DEFAULT,
        BEGIN_SUBMERSIBLE, SUBMERSIBLE_SLIDER, SUBMERSIBLE_INTAKE_OPEN, SUBMERSIBLE_INTAKE_CLOSE,
        SUBMERSIBLE_FINISH_1, SUBMERSIBLE_FINISH_2, RETURN_TO_MAIN, SPECIMENINTAKE, SPECIMENINTAKE2,
        SPECIMENINTAKE3, SPECIMENSCORE1, SPECIMENSCORE2, SPECIMENSCORE3, SPECIMENSCORE4, SPECIMENSCORE5, SPECIMENSCORE6, SPECIMENSCORE7
    }

    GlobalStateMachine globalStateMachine = GlobalStateMachine.DEFAULT;
    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;
    RoboActions robot;
    void updateTurret(stickyGamepad gp, double loopTime) {
        if(gamepad1.dpad_right) {
            turretPos -= turretSpeed * loopTime;
            if (turretPos <= Turret.MIN) {
                turretPos = Turret.MIN;
            }
        } else if (gamepad1.dpad_left) {
            turretPos += turretSpeed * loopTime;
            if (turretPos >= Turret.MAX) {
                turretPos = Turret.MAX;
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPosition = new Pose2d(0, 0, 0);
        double slideInches = 0;
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        stickyGamepad gp = new stickyGamepad(gamepad1);
        ColorSensor color = new ColorSensor(hardwareMap, "Red");
        waitForStart();
        double frequency = 0;
        double loopTime = 0;
        modules = hardwareMap.getAll(LynxModule.class);
        robot = new RoboActions(hardwareMap, startPosition);
        Drivetrain drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));

        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) CONTROL_HUB = m;
        }

        while (opModeIsActive()) {
            //read gamepads
            for (LynxModule hub : modules) {
                hub.clearBulkCache();
            }
            if (globalStateMachine != GlobalStateMachine.SUBMERSIBLE_SLIDER && globalStateMachine != GlobalStateMachine.SUBMERSIBLE_INTAKE_OPEN && globalStateMachine != GlobalStateMachine.SUBMERSIBLE_INTAKE_CLOSE) {
                if (turretPos != .5) turretPos = 0.5;
            }
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
                } else if (globalStateMachine == GlobalStateMachine.SPECIMENINTAKE) {
                    globalStateMachine = GlobalStateMachine.SPECIMENINTAKE2;
                } else if (globalStateMachine == GlobalStateMachine.SPECIMENINTAKE2) {
                    globalStateMachine = GlobalStateMachine.SPECIMENINTAKE3;
                } else if (globalStateMachine == GlobalStateMachine.SPECIMENINTAKE3) {
                    globalStateMachine = GlobalStateMachine.SPECIMENSCORE1;
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
            } else if (gp.b && (globalStateMachine == GlobalStateMachine.DEFAULT || globalStateMachine == GlobalStateMachine.INTAKE_READY)) {
                globalStateMachine = GlobalStateMachine.SPECIMENINTAKE;
            } else if (gp.b && (globalStateMachine == GlobalStateMachine.SPECIMENINTAKE)) {
                globalStateMachine = GlobalStateMachine.DEFAULT;
            }



            //state machine
            if (globalStateMachine == GlobalStateMachine.DEFAULT) { // DEFAULT
                robot.tDefault();
            } else if (globalStateMachine == GlobalStateMachine.INTAKE_READY) { //INTAKE READY
                robot.intakeReady();
            } else if (globalStateMachine == GlobalStateMachine.CLOSE_INTAKE) { //CLOSE INTAKE
                robot.closeIntake();
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.FINISHED_INTAKE;
            } else if (globalStateMachine == GlobalStateMachine.FINISHED_INTAKE) { // FINISHED INTAKE
                if (timeStamp + 250 < timer.milliseconds()) {
                    robot.finishedIntake();
                }
            } else if (globalStateMachine == GlobalStateMachine.READY_DEPOSIT) { // READY DEPOSIT
                robot.readyDeposit();
            } else if (globalStateMachine == GlobalStateMachine.SLIDES_BEGIN_SCORE) { // SLIDES BEGIN SCORE
                robot.slidesBeginScore();
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.WRIST_SCORE;
            } else if (globalStateMachine == GlobalStateMachine.WRIST_SCORE) { // WRIST SCORE
                if (timeStamp + 400 < timer.milliseconds()) {
                    robot.wristScore();
                }
            } else if (globalStateMachine == GlobalStateMachine.FINISH_SCORE) { // FINISH SCORE
                robot.finishScore();
            } else if (globalStateMachine == GlobalStateMachine.POST_SCORE) { // POST SCORE
                robot.postScore();
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.RETURN_TO_DEFAULT;
            } else if (globalStateMachine == GlobalStateMachine.RETURN_TO_DEFAULT) { // RETURN TO DEFAULT
                if (timeStamp + 450 < timer.milliseconds()) {
                    globalStateMachine = GlobalStateMachine.DEFAULT;
                }
            } else if (globalStateMachine == GlobalStateMachine.BEGIN_SUBMERSIBLE) { // BEGIN SUBMERSIBLE
                robot.beginSubmersible();
                slideInches = 0;
                globalStateMachine = GlobalStateMachine.SUBMERSIBLE_SLIDER;
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_SLIDER) { // SUBMERSIBLE SLIDER
                double increment = Math.signum(gamepad1.left_trigger - gamepad1.right_trigger) * inchesPerSecond * loopTime * Math.pow((gamepad1.left_trigger - gamepad1.right_trigger), 2);
                slideInches += increment;
                if (slideInches > 15.0) slideInches = 15.0;
                if (slideInches < 0.5) slideInches = 0.5;
                robot.submersibleSlider(slideInches);
                updateTurret(gp, loopTime);
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_INTAKE_OPEN) { // SUBMERSIBLE INTAKE OPEN
                robot.submersibleIntakeOpen();
                updateTurret(gp, loopTime);
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_INTAKE_CLOSE) { // SUBMERSIBLE INTAKE CLOSE
                robot.closeIntake();
                updateTurret(gp, loopTime);
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_FINISH_1) { // SUBMERSIBLE FINISH 1
                robot.submersibleFinish1();
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.SUBMERSIBLE_FINISH_2;
            } else if (globalStateMachine == GlobalStateMachine.SUBMERSIBLE_FINISH_2) { // SUBMERSIBLE FINISH 2
                if (timeStamp + 350 < timer.milliseconds()) {
                    robot.finishedIntake();
                    timeStamp = timer.milliseconds();
                    globalStateMachine = GlobalStateMachine.RETURN_TO_MAIN;
                }
            } else if (globalStateMachine == GlobalStateMachine.RETURN_TO_MAIN) { // RETURN TO MAIN STATE
                if (timeStamp + 350 < timer.milliseconds()) {
                    robot.returnToMain();
                    globalStateMachine = GlobalStateMachine.FINISHED_INTAKE;
                }
            } else if (globalStateMachine == GlobalStateMachine.SPECIMENINTAKE) {
                robot.specimenIntake();
            } else if (globalStateMachine == GlobalStateMachine.SPECIMENINTAKE2) {
                robot.dropSpecimenIntake();
            } else if (globalStateMachine == GlobalStateMachine.SPECIMENINTAKE3) {
                robot.finishSpecimenIntake();
            } else if (globalStateMachine == GlobalStateMachine.SPECIMENSCORE1) {
                robot.slides.setTargetSlidesPosition(0.5);
                robot.wrist.holdSpecimen();
                robot.arm.setTargetArmPosition(79);
                timeStamp = timer.milliseconds();
                globalStateMachine = GlobalStateMachine.SPECIMENSCORE2;
            } else if (globalStateMachine == GlobalStateMachine.SPECIMENSCORE2) {
                if (timeStamp + 500 < timer.milliseconds()) {
                    robot.slides.setTargetSlidesPosition(1);
                    timeStamp = timer.milliseconds();
                    globalStateMachine = GlobalStateMachine.SPECIMENSCORE3;
                }
            } else if (globalStateMachine == GlobalStateMachine.SPECIMENSCORE3) {
                if (timeStamp + 500 < timer.milliseconds()) {
                    robot.slides.setTargetSlidesPosition(9.8);
                    timeStamp = timer.milliseconds();
                    globalStateMachine = GlobalStateMachine.SPECIMENSCORE4;
                }
            } else if (globalStateMachine == GlobalStateMachine.SPECIMENSCORE4) {
                if (timeStamp + 500 < timer.milliseconds()) {
                    robot.wrist.setPosition(200); // need to fix  this shi
                    timeStamp = timer.milliseconds();
                    globalStateMachine = GlobalStateMachine.SPECIMENSCORE5;
                    robot.arm.setTargetArmPosition(45);
                }
            } else if (globalStateMachine == GlobalStateMachine.SPECIMENSCORE5) {
                if (timeStamp + 165 < timer.milliseconds()) {
                    robot.claw.open();
                    globalStateMachine = GlobalStateMachine.DEFAULT;
                }
            }






            if (gamepad1.dpad_up){
                robot.hangPower(-1);
            } else if (gamepad1.dpad_down){
                robot.hangPower(1);
            }
            else{
                robot.hangPower(0);
            }


            double newTime = getRuntime();
            loopTime = newTime - oldTime;
            frequency = 1 / loopTime;
            oldTime = newTime;


            drive.update();
            robot.drivePowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            robot.slidesUpdate();
            robot.armUpdate();
            robot.turret.setPosition(turretPos);
            gp.update();


            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", drive.getPose().getX(), drive.getPose().getY(), M.toDegrees(drive.getPose().getHeading()));
            telemetry.addData("Position", data);
            telemetry.addData("Heading Velocity: ", drive.getHeadingVelocity());
            telemetry.addData("Status", drive.getStatus());
            telemetry.addData("Hub loop Time: ", frequency);
            telemetry.addData("slides inches: ", robot.slides.getCurrentSlidesPosition());
            telemetry.addData("arm degrees:", robot.arm.getCurrentArmPosition());
            telemetry.addData("Left Trigger:", gamepad1.left_trigger);
            telemetry.addData("Right Trigger:", gamepad1.right_trigger);
            telemetry.addData("Color:", color.getColor());

            telemetry.update();
        }
    }


}
