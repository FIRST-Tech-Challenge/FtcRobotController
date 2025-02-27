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
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.TwoFishDelivery;
import org.firstinspires.ftc.teamcode.TwoFishIntake;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="TwoFish Teleop", group="AAA")
public class TwoFishTeleop extends LinearOpMode {
    DrivetrainFunctions drivetrainFunctions = null;
    TwoFishDelivery delivery = null;

    TwoFishIntake intake = null;


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
    boolean transfered = false;
    boolean dumped = false;
    boolean retracted = false;
    boolean pitched = false;

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

    private Gamepad prevGamepad2;
    @Override
    public void runOpMode() {

        delivery = new TwoFishDelivery(this, deliveryTimer);

        drivetrainFunctions = new DrivetrainFunctions(this);
        intake = new TwoFishIntake(this);

        waitForStart();

        gamepad2.copy(prevGamepad2);

        while (opModeIsActive()){

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

//                if (gamepad1.x) {
//                    drivetrainFunctions.Move(0, intake.getCenterBlockDistance("yellow")[1], 0, speedScalar);
//                }
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
                        intake.setSlidesPower(gamepad2.right_stick_y * 0.01f);
                        //intake pitch
                        intakePitchTarget += (int) (gamepad2.left_stick_y * 0.01f);
                        intake.setPitch(intakePitchTarget);

                        //Intake spin power
                        if (gamepad2.right_trigger >= 0.01) {
                            intake.setIntakePower(gamepad2.right_trigger);
                        }
                        if (gamepad2.left_trigger >= 0.01){
                            intake.setIntakePower(-gamepad2.left_trigger*0.5f);
                        }

                        //confirm intake ---> transfer
                        if(gamepad2.a && !prevGamepad2.a){
                           state = RobotState.TRANSFER;
                            intake.pitchUp();
                           intake.setSlidesTargetPosition(0);
                        }

                        //Move to grab spec ("y")
                        if(gamepad2.y && !prevGamepad2.y){
                            state = RobotState.SPECIMEN_GRAB;
                        }
                        break;

                    case TRANSFER:
                        intake.updateLength();
                        //break unless fully retracted
                        if(intake.getExtensionTicks() > 25){break;}

                        //close claw if "a" pressed
                        if(gamepad2.a && !prevGamepad2.a){
                            delivery.clawClose();
                        }

                        //Extend once the claw is closed
                        if(delivery.CheckClawClosed()){
                            delivery.resetPWM();
                            delivery.clawClose();
                            delivery.toSampleHeight();
                            delivery.setPitch(delivery.pitchSampleScorePosition);
                            state = RobotState.SAMPLE_SCORE;
                        }
                        break;

                    case SAMPLE_SCORE:
                        delivery.PControlPower(0.75);

                        //wait for "a" to score
                        if(gamepad2.a && prevGamepad2.a){
                            delivery.clawOpen();
                        }

                        //Pitch up if done scoring
                        if(delivery.CheckClawOpen()){
                            delivery.resetPWM();
                            delivery.setPitch(delivery.pitchUpPosition);
                            delivery.setSlidesTargetPosition(0);
                            state = RobotState.IDLE;
                        }

                        break;

                    case SPECIMEN_GRAB:

                        //pitch to intake position
                        delivery.setPitch(delivery.pitchIntakePosition);

                        //wait for "a" to grab
                        if(gamepad2.a && !prevGamepad2.a){
                            delivery.clawClose();
                        }

                        //Pitch up after grabbing and flip spec --- prepare to score
                        if(delivery.CheckClawClosed()){
                            delivery.setPitch(delivery.pitchUpPosition);
                            delivery.setWrist(delivery.wristDownPosition);
                            state = RobotState.SPECIMEN_SCORE;
                        }
                        break;

                    case SPECIMEN_SCORE:

                        //raise slides to spec height
                        delivery.toSpecHeight();
                        delivery.PControlPower(0.75);

                        //Hold "a" to score, press "left bumper" to let go
                        if(gamepad2.a){
                            delivery.setPitch(delivery.pitchScoreSpecPosition);
                            delivery.setWrist(delivery.wristDownPosition);
                            if(gamepad2.left_bumper && !prevGamepad2.left_bumper){
                                delivery.clawOpen();
                            }
                        } else{
                            delivery.setPitch(delivery.pitchUpPosition);
                        }

                        //flip wrist and go to idle after confirmed score
                        if(delivery.CheckClawOpen()){
                            delivery.setWrist(delivery.wristUpPosition);
                            delivery.resetPWM();
                            state = RobotState.IDLE;
                        }
                        break;

                    case IDLE:
                        delivery.setSlidesTargetPosition(0);
                        delivery.PControlPower(0.5);
                        //if left joystick or triggers are pressed, switch to intake
                        if(Math.abs(gamepad2.left_stick_y) > 0.01f || gamepad2.right_trigger > 0.01f || gamepad2.left_trigger > 0.01f){
                            state = RobotState.INTAKE;
                        }

                        //if "y" pressed go to grab spec
                        if(gamepad2.y && !prevGamepad2.y){
                            state = RobotState.SPECIMEN_GRAB;
                        }
                }

                telemetry.addData("STATE: ", state);


            }

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

//            prevPoseEstimate = poseEstimate;
            prevEncoderX = encoderX;
        }
    }
}
