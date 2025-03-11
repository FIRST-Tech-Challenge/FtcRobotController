package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.TwoFishDelivery;
import org.firstinspires.ftc.teamcode.TwoFishIntake;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="TwoFish Teleop", group="AAA")
public class TwoFishTeleop extends LinearOpMode {
    DrivetrainFunctions drivetrainFunctions = null;
    TwoFishDelivery delivery = null;

    TwoFishIntake intake = null;
    VoltageDebugger voltageDebugger = null;


    private final double HOME_X = 50;
    private final double HOME_Y = -20;

    private final double STORE_X = -35.94;
    private final double STORE_Y = 20.75;

    private double xOffset = 0.0;
    private double yOffset = 0.0;

    private Pose2d initPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
    private Pose2d prevPoseEstimate = new Pose2d(0,0,0);
    private Pose2d poseEstimate = new Pose2d(0,0,0);

    private boolean isGlobalTargeting = false;
    private double prevEncoderX = 0.0;
    private double encoderX = 0.0;
    private double deltaX = 0.0;

    private int targetExtention = 0;

    private boolean controlsRelinquished = false;
    private final double DRIVE_DEADZONE = 0.05;
    private final double SCORE_SPEED_SCALAR = 0.2;

    private double speedScalar = 1;
    private int sampleDeliveryHeight;
    private int slideTarget = 0;
    private final int HEIGHT_INCREMENT = 1;
    private final double DELIVER_PITCH_TIME = 0.2;
    private final double TRANSFER_TIME = 0.5;
    private final double DUMP_TIME = 0.25;
    private final double DELIVERY_EXTENSION_TIME = DELIVER_PITCH_TIME + 1;
    private final double SPECIMEN_SCORE_TIME = 0.75;
    private final double COLOR_SENS_CYCLE_TIME = 0.2;
    boolean transfered = false;
    boolean dumped = false;
    boolean retracted = false;
    boolean pitched = false;

    String targetSampleColor = "YELLOW";

    private float intakePitchTarget = 0;

    private enum RobotState{
        INTAKE,
        TRANSFER,
        SAMPLE_SCORE,
        SPECIMEN_GRAB,
        SPECIMEN_SCORE,
        IDLE
    }

    RobotState state = RobotState.IDLE;

    //This is the code to add rr actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime dumpTimer = new ElapsedTime();
    private ElapsedTime deliveryTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();

    private double colorSensSampleTimestamp = 0.0;

    private Gamepad prevGamepad2 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    @Override
    public void runOpMode() {

        delivery = new TwoFishDelivery(this, deliveryTimer);

        drivetrainFunctions = new DrivetrainFunctions(this);
        intake = new TwoFishIntake(this);
        voltageDebugger = new VoltageDebugger(this);

       // prevGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        waitForStart();

        while (opModeIsActive()){

            prevGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            TelemetryPacket packet = new TelemetryPacket();

            //driver 1
            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.25;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }

            if (!controlsRelinquished) {
                float leftX = gamepad1.left_stick_x;
                float leftY = gamepad1.left_stick_y;
                float rightX = gamepad1.right_stick_x;
                float rightY = gamepad1.right_stick_y;
                if (Math.abs(leftX) > DRIVE_DEADZONE || Math.abs(leftY) > DRIVE_DEADZONE || Math.abs(rightX) > DRIVE_DEADZONE || Math.abs(rightY) > DRIVE_DEADZONE*2) {
                    if(Math.abs(rightY) > DRIVE_DEADZONE*2) {
                        drivetrainFunctions.Move(leftX, rightY, rightX, speedScalar);
                    }else{
                        drivetrainFunctions.Move(leftX, leftY, rightX, speedScalar);
                    }
                } else {
                    drivetrainFunctions.Stop();
                }

//                if (gamepad1.y) {
//                    drivetrainFunctions.Move(0, intake.getCenterBlockDistance("yellow")[1], 0, speedScalar);
//                }
                if(gamepad1.b){
                    targetSampleColor = "RED";
                }
                if(gamepad1.x){
                    targetSampleColor = "BLUE";
                }
                if(gamepad1.y){
                    targetSampleColor = "YELLOW";
                }
                if(gamepad1.a){
                    targetSampleColor = "NONE";
                }
            }

            //driver 2
            if(!controlsRelinquished) {
                float leftX = gamepad2.left_stick_x;
                float leftY = -gamepad2.left_stick_y;
                float rightX = gamepad2.right_stick_x;
                float rightY = -gamepad2.right_stick_y;



                if(gamepad2.back){
                    state = RobotState.IDLE;
                }

                switch(state) {

                    case INTAKE:
                        //intake extension
                        intake.setSlidesPower(0.75);
                        intake.setSlidesTargetPosition(intake.getExtensionTicks() - (int)(gamepad2.left_stick_y * 200));
                        //intake pitch
                        intakePitchTarget = (float) Math.min(intake.downPitch, Math.max(intake.upPitch, intakePitchTarget)); //down pitch is larger limit
                        intakePitchTarget += (gamepad2.right_stick_y * 0.01f);
                        intake.setPitch(intakePitchTarget);

                        //Color sensor

                        if(intakeTimer.seconds() - colorSensSampleTimestamp > COLOR_SENS_CYCLE_TIME){
                            colorSensSampleTimestamp = intakeTimer.seconds();
                            if(intake.verifyColor(targetSampleColor)){
                                intake.pitchUp();
                                intakePitchTarget = (float)intake.upPitch;
                            }
                            if(targetSampleColor.equals("RED")){
                                if(intake.posessedSampleColor.equals("BLUE")){
                                    intake.setIntakePower(-0.5f);
                                    intake.setTransferPower(1);
                                }
                            }
                            if(targetSampleColor.equals("BLUE")){
                                if(intake.posessedSampleColor.equals("RED")){
                                    intake.setIntakePower(-0.5f);
                                    intake.setTransferPower(1);
                                }
                            }
                        }
                        //Intake spin power
                        if (gamepad2.right_trigger >= 0.01) {
                            intake.setIntakePower(gamepad2.right_trigger);
                        }
                        if (gamepad2.left_trigger >= 0.01){
                            intake.setIntakePower(-gamepad2.left_trigger*0.5f);
                        }
                        if(gamepad2.left_trigger < 0.01 && gamepad2.right_trigger < 0.01){
                            intake.setIntakePower(0);
                        }

                        //confirm intake ---> transfer
                        if(currentGamepad2.a && !prevGamepad2.a){
                           state = RobotState.TRANSFER;
                            intake.pitchUp();
                            intake.setTransferPower(-1.0f);
                           intake.toMinExtention();
                            delivery.setWrist(delivery.wristUpPosition);
                            delivery.toSpecHeight();
                            delivery.setPitch(delivery.pitchTransferPosition);
                        }

                        //Move to grab spec ("y")
                        if(currentGamepad2.y && !prevGamepad2.y){
                            delivery.setWrist(delivery.wristUpPosition);
                            intake.setTransferPower(0.0f);
                            delivery.setClawPosition(delivery.clawOpenPosition);
                            delivery.setPitch(delivery.pitchIntakePosition);
//                            delivery.resetPWM();
                            state = RobotState.SPECIMEN_GRAB;
                        }
                        break;

                    case TRANSFER:

                        if(delivery.CheckIfDonePitching(1)){
                            delivery.toTransferHeight();
                        }

                        delivery.setSlidesPower(0.5);
//                        delivery.stopSlidesIfStuck();
                        intake.setSlidesPower(0.5);
                        //break unless fully retracted
                        if(intake.getExtensionTicks() > 25+intake.minExtension){break;}

                        //close claw if "a" pressed
                        if(currentGamepad2.a && !prevGamepad2.a){
                            intake.setTransferPower(0.0f);
                            delivery.clawClose();
                        }

                        //Extend once the claw is closed
                        if(delivery.CheckClawClosed()){
//                            delivery.resetPWM();
                            delivery.clawClose();
                            delivery.toSampleHeight();
                            state = RobotState.SAMPLE_SCORE;
                        }
                        break;

                    case SAMPLE_SCORE:

                        // manual control of slide height
                        if(Math.abs(gamepad2.left_stick_y) > 0.01){
                            delivery.slideTarget = delivery.getSlideHeight() - gamepad2.left_stick_y * 200;
                            delivery.setSlidesTargetPosition((int)delivery.slideTarget);
                        }

                        delivery.setSlidesPower(1);

                        if(delivery.getSlideHeight() > delivery.specHeight){
                            delivery.setPitch(delivery.pitchSampleScorePosition);
                        }

                        //wait for "a" to score
                        if(gamepad2.a && prevGamepad2.a){
                            delivery.clawOpen();
                        }

                        //Pitch up if done scoring
                        if(delivery.CheckClawOpen()){
//                            delivery.resetPWM();
                            delivery.setPitch(delivery.pitchUpPosition);
                            delivery.toMinHeight();
                            state = RobotState.IDLE;
                        }

                        break;

                    case SPECIMEN_GRAB:
                        delivery.toMinHeight();
                        delivery.setSlidesPower(0.5);

                        delivery.setWrist(delivery.wristUpPosition);

                        if(delivery.CheckIfDonePitching()){
                            delivery.setWrist(delivery.wristUpPosition);
                        }

                        //wait for "a" to grab
                        if(currentGamepad2.a && !prevGamepad2.a){
                            delivery.clawClose();
                        }

                        //Pitch up after grabbing and flip spec --- prepare to score
                        if(delivery.CheckClawClosed()){
                            delivery.setPitch(delivery.pitchUpPosition);
                            delivery.toSpecHeight();
                            delivery.setWrist(delivery.wristDownPosition);
                            state = RobotState.SPECIMEN_SCORE;
                        }
                        break;

                    case SPECIMEN_SCORE:
                        voltageDebugger.addActionVoltageTelemetry();

                        // manual control of slide height
                        if(Math.abs(gamepad2.left_stick_y) > 0.01){
                            delivery.slideTarget = delivery.getSlideHeight() - gamepad2.left_stick_y * 200;
                            delivery.setSlidesTargetPosition((int)delivery.slideTarget);
                        }

                        if(currentGamepad2.dpad_up && !prevGamepad2.dpad_up){
                            delivery.slideTarget = delivery.getSlideHeight() + 400;
                            delivery.setSlidesTargetPosition((int)delivery.slideTarget);
                        }

                        if(currentGamepad2.dpad_down && !prevGamepad2.dpad_down){
                            delivery.slideTarget = delivery.getSlideHeight() - 400;
                            delivery.setSlidesTargetPosition((int)delivery.slideTarget);
                        }
                        delivery.setSlidesPower(0.5);

                        //lb to score
                        if(gamepad2.left_bumper){
                            delivery.clawOpen();
                        }

                        //falling a --> score
                        if(currentGamepad2.a && !prevGamepad2.a){
                            //pitch more if slides are higher
                            if(delivery.getSlideHeight() > delivery.specHeight + 300){
                                delivery.setPitch(delivery.pitchScoreSpecPosition - 0.1);
                            }else{
                                delivery.setPitch(delivery.pitchScoreSpecPosition);
                            }
                            delivery.setWrist(delivery.wristDownPosition);
                        }

                        //rising a --> up
                        if(!currentGamepad2.a && prevGamepad2.a){
                            delivery.setPitch(delivery.pitchUpPosition);
                            voltageDebugger.recordActionVoltage();
                        }

                        //flip wrist and go to idle after confirmed score
                        if(delivery.CheckClawOpen()){
                            delivery.setWrist(delivery.wristUpPosition);
                            delivery.setPitch(delivery.pitchUpPosition);
//                            delivery.resetPWM();
                            state = RobotState.IDLE;
                        }
                        intake.pitchUp();
                        break;

                    case IDLE:
                        delivery.toSpecHeight();
                        delivery.setSlidesPower(0);

                        //if left joystick or triggers are pressed, switch to intake
                        if(Math.abs(gamepad2.left_stick_y) > 0.01f || gamepad2.right_trigger > 0.01f || gamepad2.left_trigger > 0.01f){
                            state = RobotState.INTAKE;
                        }

                        //if "y" pressed go to grab spec
                        if(currentGamepad2.y && !prevGamepad2.y){
//                            delivery.resetPWM();
                            delivery.setClawPosition(delivery.clawOpenPosition);
                            delivery.setPitch(delivery.pitchIntakePosition);
                            state = RobotState.SPECIMEN_GRAB;
                        }
                }

//                telemetry.addData("Intake Pitch Target: ", intakePitchTarget);
                telemetry.addData("STATE: ", state);
                delivery.addSlideTelemetry();


            }

            telemetry.addData("Is intake limit switch true?", intake.limitSwitch.isPressed());
            telemetry.addData("Is delivery limit switch true?", delivery.limitSwitch.isPressed());
            telemetry.update();

            //Adds rr actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            intake.limitCheck();
            delivery.limitCheck();
            telemetry.addData("intake currect extension: ", intake.getExtensionTicks());
            telemetry.addData("intake min extension: ", intake.minExtension);
            telemetry.addData("delivery currect extension: ", delivery.getSlideHeight());
            telemetry.addData("delivery min extension: ", delivery.minHeight);
//            prevPoseEstimate = poseEstimate;
            prevEncoderX = encoderX;
        }
    }
}
