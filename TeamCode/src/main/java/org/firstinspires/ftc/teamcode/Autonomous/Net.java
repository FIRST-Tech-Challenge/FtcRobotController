package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.OneFishIntake;
import org.firstinspires.ftc.teamcode.OneFishSampleDelivery;
import org.firstinspires.ftc.teamcode.OneFishSpecimenDelivery;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Net Onefish", group = "Autonomous")
public class Net extends LinearOpMode {
    OneFishSpecimenDelivery specimenDelivery = null;
    OneFishIntake intake = null;
    OneFishSampleDelivery sampleDelivery = null;

    DrivetrainFunctions driveTrain = null;

    private static final int VERTICAL_INTAKE_POS = -330;
    private static final int CLAW_FLOOR = 0;
    private static final int CLAW_COLLECT = 800;
    private static final int CLAW_SCORE = 1650;
    private static final int CLAW_HIGH_RUNG = 2200;

    private boolean imuInitialized = false;

    @Override
    public void runOpMode() {
        driveTrain = new DrivetrainFunctions(this, true);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -62.12, Math.toRadians(90)));


        specimenDelivery = new OneFishSpecimenDelivery(this);
        intake = new OneFishIntake(this);
        sampleDelivery = new OneFishSampleDelivery(this, true);

        Action trajToNet;
        Action trajToSampleOne;
        Action returnToNet;
        Action returnToNet2;
        Action turnToSampleOne;
        Action trajToSampleTwo;
        Action trajToSampleThree;
        Action returnToNet3;
        Action turnToSampleTwo;
        Action turnToDeliverSampleOne;
        Action turnToIntakeSampleTwo;
        Action turnToDeliverSampleTwo;
        Action turnToIntakeSampleThree;
        Action turnToDeliverSampleThree;
        Action trajToPark;


        trajToNet = drive.actionBuilder(new Pose2d(-36, -62.12, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-65, -55), Math.toRadians(10))
                .build();

        trajToSampleOne = drive.actionBuilder(new Pose2d(-65, -53, Math.toRadians(79)))
                .strafeToLinearHeading(new Vector2d(-55, -46.5), Math.toRadians(79), new AngularVelConstraint(6))
                .build();

        turnToSampleOne = drive.actionBuilder(new Pose2d(-65, -53, Math.toRadians(10)))
                .turnTo(Math.toRadians(79))
                .build();

        returnToNet = drive.actionBuilder(new Pose2d(-55, -47, Math.toRadians(77)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-66, -60, Math.toRadians(10)), Math.toRadians(0))
                .build();

        trajToSampleTwo = drive.actionBuilder(new Pose2d(-66, -60, Math.toRadians(10)))
                .strafeToLinearHeading(new Vector2d(-58, -51), Math.toRadians(115), new AngularVelConstraint(7))
                .build();

        turnToSampleTwo = drive.actionBuilder(new Pose2d(-63, -53, Math.toRadians(55)))
                .turnTo(Math.toRadians(90))
                .build();

        returnToNet2 = drive.actionBuilder(new Pose2d(-58, -51, Math.toRadians(115)))
                .strafeToLinearHeading(new Vector2d(-57.5, -57), Math.toRadians(30))
                .build();

        trajToSampleThree = drive.actionBuilder(new Pose2d(-57.5, -57, Math.toRadians(30)))
                .strafeToLinearHeading(new Vector2d(-55, -48), Math.toRadians(130), new AngularVelConstraint(9))
                .build();



        returnToNet3 = drive.actionBuilder(new Pose2d(-58, -48, Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(-57.5, -57), Math.toRadians(30))
                .build();

        trajToPark = drive.actionBuilder(new Pose2d(-62, -55, Math.toRadians(0)))
                .strafeTo(new Vector2d(-40, -55))
                .strafeTo(new Vector2d(-40, -10))
                .strafeTo(new Vector2d(-24, -10))
                .build();


        waitForStart();

        if (isStopRequested()) return;
        intake.pitchToTransfer();
        sampleDelivery.pitchToVertical();
        sleep(200);
        sampleDelivery.pitchToTransfer();
        Actions.runBlocking(new ParallelAction(trajToNet, TransferSampleAction(-2050)));
        sleep(350);
        Actions.runBlocking(new ParallelAction(DeliverSampleHighAction(), intake.RunToLengthAction(1200, 500)));
        intake.pitchUp();
        Actions.runBlocking(intake.SpinIntakeAction(0.5, 550));//sleep(550);
        intake.pitchToTransfer();
        sleep(100);
        Actions.runBlocking(new ParallelAction(ResetSampleAction(), turnToSampleOne));
//        intake.pitchDown();
        Actions.runBlocking(new ParallelAction(trajToSampleOne, intake.RunToLengthAction(2000, 500), intake.SpinIntakeAction(-1,1000)));
        intake.pitchDown();
        Actions.runBlocking(intake.SpinIntakeAction(-1,1000));
        intake.pitchToTransfer();
        Actions.runBlocking(new ParallelAction(returnToNet, intake.RunToLengthAction(-5, 900), intake.SpinIntakeAction(-1, 500)));
        intake.pitchToTransfer();
        sleep(100);
        Actions.runBlocking(TransferSampleAction(-2050));
        sleep(350);
        Actions.runBlocking(new ParallelAction(DeliverSampleHighAction(), intake.RunToLengthAction(1200, 500)));
        intake.pitchUp();
        Actions.runBlocking(intake.SpinIntakeAction(0.5, 550));//sleep(550);
        intake.pitchToTransfer();
//        intake.pitchDown();
//        Actions.runBlocking(turnToSampleTwo);
        Actions.runBlocking(new ParallelAction(ResetSampleAction(), trajToSampleTwo, intake.RunToLengthAction(1300, 500), intake.SpinIntakeAction(-1,1000)));
        intake.pitchDown();
        Actions.runBlocking(intake.SpinIntakeAction(-1,1000));
        intake.pitchToTransfer();
        Actions.runBlocking(new ParallelAction(returnToNet, intake.RunToLengthAction(-5, 900), intake.SpinIntakeAction(-1, 500)));
        intake.pitchToTransfer();
        sleep(200);
        Actions.runBlocking(new ParallelAction(TransferSampleAction(-1200), returnToNet2));
        Actions.runBlocking(new ParallelAction(DeliverSampleAction(), intake.RunToLengthAction(1200, 500)));
        intake.pitchUp();
        Actions.runBlocking(intake.SpinIntakeAction(0.5, 550));//sleep(550);
        intake.pitchToTransfer();
        Actions.runBlocking(new ParallelAction(ResetSampleAction(), trajToSampleThree, intake.RunToLengthAction(2000, 500), intake.SpinIntakeAction(-1,1000)));
        intake.pitchDown();
        Actions.runBlocking(intake.SpinIntakeAction(-1,1000));
        intake.pitchToTransfer();
        Actions.runBlocking(new ParallelAction(returnToNet, intake.RunToLengthAction(-5, 900), intake.SpinIntakeAction(-1, 500)));

        sleep(200);
        Actions.runBlocking(new ParallelAction(TransferSampleAction(-1200), returnToNet3));
        Actions.runBlocking(new ParallelAction(DeliverSampleAction(), intake.RunToLengthAction(-5, 500)));
        Actions.runBlocking(ResetSampleAction());
        //sampleDelivery.setSlidesTargetPosition(0);
        //                        sampleDelivery.PControlPower(1);
        //                        if(timer.seconds()>DELIVER_PITCH_TIME){
        //                            if(!transfered){
        //                                intake.pitchToTransfer();
        //                                intake.setIntakePower(-0.25f);
        //                            }
        //                            if(gamepad2.a){
        //                                sampleDelivery.clawOpen();
        //                                transferTimer.reset();
        //                                intake.setIntakePower(0);
        //                                transfered = true;
        //                            }
        //                        }
        //                        telemetry.addData("transferred: ", transfered);
        //                        if(transferTimer.seconds() > TRANSFER_TIME && transfered){
        //                            intake.pitchDown();
        //                            //transfered = false;
        //                            sampleDeliveryHeight = -2090;
        //                            state = RobotState.DELIVERY_EXTEND;
        //                            timer.reset();
        //                        }

//        Actions.runBlocking(trajToPark);


    }
    public class TransferSampleRR implements Action {
        private boolean initialized = false;
        private int targetHeight;
        private ElapsedTime timer = new ElapsedTime();
        private final double PREP_TIME = 250*2;
        private final double GRAB_TIME = 250*2;
        private final double CLEARANCE_TIME = 250*2;



        public TransferSampleRR(int TargetHeight) {
            targetHeight = TargetHeight;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                sampleDelivery.setSlidesTargetPosition(targetHeight);
                initialized = true;

            }

            if (timer.milliseconds() < PREP_TIME) {
                sampleDelivery.setSlidesTargetPosition(5);
                sampleDelivery.PControlPower(3);
                intake.pitchToTransfer();
                intake.setIntakePower(-0.5f);
                sampleDelivery.pitchToTransfer();
                sampleDelivery.clawClose();
                return true;
            } else if (timer.milliseconds() > PREP_TIME && timer.milliseconds() < PREP_TIME + GRAB_TIME) {
                sampleDelivery.clawOpen();
                return true;
            } else if ((timer.milliseconds() > PREP_TIME + GRAB_TIME && timer.milliseconds() < PREP_TIME + GRAB_TIME + CLEARANCE_TIME)) {
                sampleDelivery.setSlidesTargetPosition(targetHeight);
                sampleDelivery.PControlPower(3);
                return true;
            } else {
                sampleDelivery.pitchToVertical();
                return false;
            }
        }
    }
    public Action TransferSampleAction(int heightInTicks) {
        return new Net.TransferSampleRR(heightInTicks);
    }

    public class DeliverSampleRR implements Action {
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        private final double PREP_TIME = 250*2;
        private final double DROP_TIME = 250*2;
        private final double CLEARANCE_TIME = 250*2;

        public DeliverSampleRR() {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                timer.reset();
            }

            if (timer.milliseconds() < PREP_TIME) {
                intake.pitchToTransfer();
                sampleDelivery.clawOpen();
                sampleDelivery.pitchToShake();
                return true;
            } else if (timer.milliseconds() > PREP_TIME && timer.milliseconds() < PREP_TIME + DROP_TIME) {
                sampleDelivery.clawClose();
                return true;
            } else if ((timer.milliseconds() > PREP_TIME + DROP_TIME && timer.milliseconds() < PREP_TIME + DROP_TIME + CLEARANCE_TIME)) {
                sampleDelivery.pitchToVertical();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action DeliverSampleAction() {
        return new Net.DeliverSampleRR();
    }

    public class DeliverSampleHighRR implements Action {
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        private final double PREP_TIME = 250*2;
        private final double DROP_TIME = 250*2;
        private final double CLEARANCE_TIME = 250*2;

        public DeliverSampleHighRR() {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                timer.reset();
            }

            if (timer.milliseconds() < PREP_TIME) {
                intake.pitchToTransfer();
                sampleDelivery.clawOpen();
                sampleDelivery.pitchToDeliver();
                return true;
            } else if (timer.milliseconds() > PREP_TIME && timer.milliseconds() < PREP_TIME + DROP_TIME) {
                sampleDelivery.clawClose();
                return true;
            } else if ((timer.milliseconds() > PREP_TIME + DROP_TIME && timer.milliseconds() < PREP_TIME + DROP_TIME + CLEARANCE_TIME)) {
                sampleDelivery.pitchToVertical();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action DeliverSampleHighAction() {
        return new Net.DeliverSampleHighRR();
    }

    public class ResetSampleRR implements Action {
        private boolean initialized = false;
        private int targetHeight;
        private ElapsedTime timer = new ElapsedTime();
        private final double PREP_TIME = 250*2;
        private final double LOWER_TIME = 500*2;

        public ResetSampleRR() {
            targetHeight = 0;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                sampleDelivery.setSlidesTargetPosition(targetHeight);
                initialized = true;
                timer.reset();
            }

            if (timer.milliseconds() < PREP_TIME) {
//                intake.pitchToTransfer();
                sampleDelivery.clawClose();
                sampleDelivery.pitchToTransfer();
                return true;
            } else if (timer.milliseconds() > PREP_TIME && timer.milliseconds() < PREP_TIME + LOWER_TIME) {
                sampleDelivery.PControlPower(3);
                return true;
            } else {
                return false;
            }
        }
    }
    public Action ResetSampleAction() {
        return new Net.ResetSampleRR();
    }
}