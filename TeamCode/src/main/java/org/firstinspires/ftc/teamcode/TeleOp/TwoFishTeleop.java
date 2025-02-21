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
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;
import org.firstinspires.ftc.teamcode.TwoFishDelivery;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="TwoFish Teleop", group="AAA")
public class TwoFishTeleop extends LinearOpMode {
    DrivetrainFunctions drivetrainFunctions = null;
    TwoFishDelivery delivery = null;

    OneFishIntake intake = null;


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
    private final double INTAKE_EXTENSION_TIME = 0.25;
    private final double DELIVER_PITCH_TIME = 0.2;
    private final double PITCH_TO_DELIVER_TIME = 1;
    private final double SHAKE_TIME = 1.5;
    private final double TRANSFER_TIME = 0.5;
    private final double DUMP_TIME = 0.25;
    private final double DELIVERY_EXTENSION_TIME = DELIVER_PITCH_TIME + 1;
    private final double SPECIMEN_SCORE_TIME = 0.75;
    boolean transfered = false;
    boolean dumped = false;
    boolean retracted = false;
    boolean pitched = false;

    private enum RobotState{
        INTAKE_EXTEND,
        INTAKE,
        TRANSFER,
        DELIVERY_EXTEND,
        DELIVERY_DUMP,
        SPECIMEN,
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
        intake = new OneFishIntake(this);

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
//                        intake.slideTarget += (int) (gamepad2.right_stick_y *= 0.01f);

                        //Intake spin power
                        if (gamepad2.right_trigger >= 0.01) {
                            intake.setIntakePower(gamepad2.right_trigger);
                        }
                        if (gamepad2.left_trigger >= 0.01){
                            intake.setIntakePower(-gamepad2.left_trigger);
                        }

                        //confirm intake ---> transfer
                        if(gamepad2.a && prevGamepad2.a){
                           state = RobotState.TRANSFER;
//                           intake.slideTarget = 0;
                        }
                        break;

                    case TRANSFER:
                        intake.pitchToTransfer();
//                        intake.slideTarget = 0;

                        //break unless fully retracted
                        if(intake.getExtensionTicks() > 25){break;}


                        //close claw if "a" pressed
                        if(gamepad2.a && prevGamepad2.a){
                            delivery.clawClose();
                        }

                        //Extend once the claw is closed
                        if(delivery.CheckClawClosed()){
                            delivery.toSpecHeight();
                            delivery.setPitch(delivery.pitchUpPosition);
                            state = RobotState.DELIVERY_EXTEND;
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
