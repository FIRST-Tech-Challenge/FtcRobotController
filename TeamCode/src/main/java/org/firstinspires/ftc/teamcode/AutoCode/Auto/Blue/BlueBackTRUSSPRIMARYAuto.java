package org.firstinspires.ftc.teamcode.AutoCode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoCode.Auto.Spike;
import org.firstinspires.ftc.teamcode.AutoCode.Control.AcceleratedGain;
import org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base.StateMPointApproach;
import org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base.StateMTrajectory;
import org.firstinspires.ftc.teamcode.AutoCode.Trajectory.LiftStateMAuto;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.drivebase.CenterStageDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.StateM.LiftDownStateM;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Globals.FirstMoving;
import static org.firstinspires.ftc.teamcode.Globals.WeHaveNoGoods;
import static org.firstinspires.ftc.teamcode.Globals.X;
import static org.firstinspires.ftc.teamcode.Globals.Y;
import static org.firstinspires.ftc.teamcode.Globals.trackingWheelIntegrator;
import static org.firstinspires.ftc.teamcode.Globals.wheelH;


@Autonomous(preselectTeleOp = "MecDrive")
public class BlueBackTRUSSPRIMARYAuto extends LinearOpMode {

//    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

    CenterStageDriveBase centerStageDriveBase;

    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx RL;
    private DcMotorEx RR;

    private DcMotorEx rightHook;
    private DcMotorEx leftHook;
    private DcMotorEx intake;
    private DcMotorEx Lift;
    private DcMotorEx RHang;
    private DcMotorEx LHang;
    private Servo RHook;
    private Servo LHook;
    private Servo intakeLift;
    private Servo airplane;
    private Servo Door;
    private Servo RLock;
    private Servo LLock;
    private Servo Pivot;
    private Servo SLift;

    private float Rservopos;
    private float Lservopos;

    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private int LiftCounts;
    private float speedFactor;

    OpenCvCamera openCvCamera;

    private Spike Decision;
    private static DcMotorEx LeftTW;
    private static DcMotorEx RightTW;
    private static DcMotorEx BackTW;
//    public static double Y;
//    public static double X;
//    public static double wheelH;

    LiftStateMAuto LSMA = new LiftStateMAuto();
    LiftDownStateM LDSM = new LiftDownStateM();



    //      INT STATEM
        StateMTrajectory CenterSpike;
        StateMTrajectory LeftSpike;
        StateMTrajectory RightSpike;
        StateMTrajectory RightPark;
        StateMTrajectory CenterPark;
        StateMTrajectory LeftPark;



//        StateMTrajectory FirstMovement;
//        StateMTrajectory FirstMovement;
//


//      INT Movment Trajectory



    //Basics For Tracking
        MovingStatistics movingStatistics = new MovingStatistics(300);
//        static LynxDcMotorController ctrl;
//        LynxModule module;
        CenterStageDriveBase PowerPlayDrivebase;
        StateMTrajectory trajectory;




        @Override
        public void runOpMode() throws InterruptedException
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            BlueSpikeControllerABCs FrogFinderPipeline = new BlueSpikeControllerABCs();
            openCvCamera.setPipeline(FrogFinderPipeline);
            openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    /*
                     * Tell the camera to start streaming images to us! Note that you must make sure
                     * the resolution you specify is supported by the camera. If it is not, an exception
                     * will be thrown.
                     *
                     * Also, we specify the rotation that the camera is used in. This is so that the image
                     * from the camera sensor can be rotated such that it is always displayed with the image upright.
                     * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                     * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                     * away from the user.
                     */
                    openCvCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                    openCvCamera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                }
                public void onError(int errorCode) {

                }
            });
            trackingWheelIntegrator = new TrackingWheelIntegrator();

            FL= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "FL");
            FR= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "FR");
            RL= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RL");
            RR= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RR");

            intake= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "intake");
            Globals.Lift = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "Lift");
            LHang = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "LHang");
            RHang = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RHang");


            intakeLift=(Servo) hardwareMap.get(Servo.class, "intakeLift");
            airplane=(Servo)  hardwareMap.get(Servo.class, "airplane");
            Globals.Door=(Servo)  hardwareMap.get(Servo.class, "Door");
            RLock=(Servo)  hardwareMap.get(Servo.class, "RLock");
            LLock=(Servo)  hardwareMap.get(Servo.class, "LLock");
            Globals.Pivot=(Servo)  hardwareMap.get(Servo.class, "Pivot");
            Globals.SLift=(Servo)  hardwareMap.get(Servo.class, "SLift");
            RHook=(Servo)  hardwareMap.get(Servo.class, "RHook");
            LHook=(Servo)  hardwareMap.get(Servo.class, "LHook");

            LeftTW = hardwareMap.get(DcMotorEx.class, "LHang");
            RightTW = hardwareMap.get(DcMotorEx.class, "RHang");
            BackTW = hardwareMap.get(DcMotorEx.class, "intake");


            centerStageDriveBase = new CenterStageDriveBase();
            centerStageDriveBase.init(hardwareMap);
            centerStageDriveBase.resetEncoders();
            centerStageDriveBase.enableBrake(true);
            centerStageDriveBase.enablePID();
            Globals.robot=centerStageDriveBase;
            Globals.driveBase=centerStageDriveBase;
            Globals.opMode = this;
            trackingWheelIntegrator = trackingWheelIntegrator;

            telemetry.setMsTransmissionInterval(20);

            telemetry.setMsTransmissionInterval(20);

            //Setting Start Location in Auto.
            trackingWheelIntegrator.setFirstTrackingVal(120,0);

            clearEnc();

            while (!isStarted() && !isStopRequested())
            {
                //Vision During INT
                WeHaveNoGoods = false;
                FirstMoving = true;
                telemetry.addData("Position", FrogFinderPipeline.GetPosition());
                telemetry.update();
                Decision = FrogFinderPipeline.GetPosition();
            }

    buildTrajectory(); //Command for State Mechine.

            while (opModeIsActive()) {
                Globals.updateTracking();
                int left = LeftTW.getCurrentPosition();
                int right = RightTW.getCurrentPosition();
                int aux = BackTW.getCurrentPosition();
                trackingWheelIntegrator.update(left, right, aux);

                Y = trackingWheelIntegrator.getY();
                X = trackingWheelIntegrator.getX();
                wheelH = trackingWheelIntegrator.getHeading();

                telemetry.addData("X", X);
                //opMode.telemetry.addData("FrontInch", inch);
                telemetry.addData("Y", Y);
                telemetry.addData("wheelH", wheelH);
                telemetry.update();

                if (Decision == Spike.A) {
                    LeftSpike.followInteration();
                    if (Y < -30) {
                        LSMA.runIteration();
                    }
                    if (WeHaveNoGoods) {
                        LDSM.runIteration();
                    }
                }
                if (Decision == Spike.B) {
                    CenterSpike.followInteration();
                    if (Y < -100) {
                        LSMA.runIteration();
                    }
                    if (WeHaveNoGoods) {
                        LDSM.runIteration();
                    }
                }
                if (Decision == Spike.C) {
                    RightSpike.followInteration();
//                    if (Y < -60) {
//                        LSMA.runIteration();
//                    }
                    if (WeHaveNoGoods) {
                        LDSM.runIteration();
                    }
                }

            }

        }

    public static void clearEnc()        {
            LeftTW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightTW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackTW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftTW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightTW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackTW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

        public void buildTrajectory() {


            LeftSpike = new StateMTrajectory.Builder()

//                    .addMovement(new StateMPointApproach.Builder()
//                            .setTargetPosition(24,-24)
//                            .setMaxPower(.35) // .4 - 6
//                            .setXyGain(.04)
//                            .setTargetHeading(0)
//                            .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                            .setMaxTurnPower(.2)
//                            .setMovementThresh(1)
//                            .setHeadingThreshold(2)
//                            .stopMotorsOnDone(true)
//                            .build())
//
//                    .addMovement(new StateMPointApproach.Builder()
//                            .setTargetPosition(42,-27)
//                            .setMaxPower(.35) // .4 - 6
//                            .setXyGain(.04)
//                            .setTargetHeading(0)
//                            .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                            .setMaxTurnPower(.2)
//                            .setMovementThresh(1)
//                            .setHeadingThreshold(2)
//                            .stopMotorsOnDone(true)
//                            .build())
//
//                    .addMovement(new StateMPointApproach.Builder() //First movment.
//                            .setTargetPosition(42,-23)
//                            .setMaxPower(.4) // .4 - 6
//                            .setXyGain(.04)
//                            .setTargetHeading(0)
//                            .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                            .setMaxTurnPower(.2)
//                            .setMovementThresh(2)
//                            .setHeadingThreshold(2)
//                            .stopMotorsOnDone(true)
//                            .build())
//
//                    .addMovement(new StateMPointApproach.Builder() //First movment.
//                            .setTargetPosition(24,-24)
//                            .setMaxPower(.4) // .4 - 6
//                            .setXyGain(.04)
//                            .setTargetHeading(0)
//                            .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                            .setMaxTurnPower(.2)
//                            .setMovementThresh(2)
//                            .setHeadingThreshold(2)
//                            .stopMotorsOnDone(true)
//                            .build())
//
//                    .addMovement(new StateMPointApproach.Builder()
//                            .setTargetPosition(24,-50)
//                            .setMaxPower(.15) // .4 - 6
//                            .setXyGain(.03)
//                            .setTargetHeading(0)
//                            .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                            .setMaxTurnPower(.2)
//                            .setMovementThresh(1)
//                            .setHeadingThreshold(2)
//                            .stopMotorsOnDone(true)
//                            .build())
//
//                    .addMovement(new StateMPointApproach.Builder()
//                            .setTargetPosition(36,-71)
//                            .setMaxPower(.15) // .4 - 6
//                            .setXyGain(.03)
//                            .setTargetHeading(-90)
//                            .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                            .setMaxTurnPower(.2)
//                            .setMovementThresh(1)
//                            .setHeadingThreshold(2)
//                            .stopMotorsOnDone(true)
//                            .build())
//
//                    .addMovement(new StateMPointApproach.Builder()
//                            .setTargetPosition(-28,-48)
//                            .setMaxPower(.15) // .4 - 6
//                            .setXyGain(.03)
//                            .setTargetHeading(-90)
//                            .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                            .setMaxTurnPower(.2)
//                            .setMovementThresh(1)
//                            .setHeadingThreshold(2)
//                            .stopMotorsOnDone(true)
//                            .build())

                    .build();

        CenterSpike = new StateMTrajectory.Builder()

                .addMovement(new StateMPointApproach.Builder()
                        .setTargetPosition(113,0)    //50.2
                        .setMaxPower(.2) // .4 - 6
                        .setXyGain(.04)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.2)
                        .setMovementThresh(1)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(113,-30)
                        .setMaxPower(.2) // .4 - 6
                        .setXyGain(.05)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.2)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder()
                        .setTargetPosition(113,-23)
                        .setMaxPower(.2) // .4 - 6
                        .setXyGain(.05)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.2)
                        .setMovementThresh(1)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(96,-23)
                        .setMaxPower(.4) // .4 - 6.
                        .setXyGain(.03)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.5)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(96,-48)
                        .setMaxPower(.4) // .4 - 6.
                        .setXyGain(.03)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.5)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(108,-67)
                        .setMaxPower(.4) // .4 - 6.
                        .setXyGain(.03)
                        .setTargetHeading(-88)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.5)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(90,-70)
                        .setMaxPower(.4) // .4 - 6.
                        .setXyGain(.03)
                        .setTargetHeading(-88)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.5)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(52,-81)
                        .setMaxPower(.4) // .4 - 6.
                        .setXyGain(.03)
                        .setTargetHeading(-88)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.5)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(11,-95)
                        .setMaxPower(.4) // .4 - 6.
                        .setXyGain(.03)
                        .setTargetHeading(-90)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.5)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .build();

        RightSpike = new StateMTrajectory.Builder()

                .addMovement(new StateMPointApproach.Builder()
                        .setTargetPosition(109,0)
                        .setMaxPower(.35) // .4 - 6
                        .setXyGain(.04)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.3)
                        .setMovementThresh(1)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(104,-25)
                        .setMaxPower(.4) // .4 - 6
                        .setXyGain(.04)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.2)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(112,-19)
                        .setMaxPower(.4) // .4 - 6
                        .setXyGain(.04)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.2)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())

                .addMovement(new StateMPointApproach.Builder() //First movment.
                        .setTargetPosition(122,-19)
                        .setMaxPower(.4) // .4 - 6
                        .setXyGain(.04)
                        .setTargetHeading(0)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
                        .setMaxTurnPower(.2)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
//
//                .addMovement(new StateMPointApproach.Builder() //First movment.
//                        .setTargetPosition(123,-52)
//                        .setMaxPower(.4) // .4 - 6
//                        .setXyGain(.04)
//                        .setTargetHeading(0)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                        .setMaxTurnPower(.2)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(2)
//                        .stopMotorsOnDone(true)
//                        .build())
                .build();
//
//        RightPark = new StateMTrajectory.Builder()
//
//                .addMovement(new StateMPointApproach.Builder() //First movment.
////                        Center
////                      .setTargetPosition(36,-67)
////                        LeftSpike
////                        .setTargetPosition(51,-46)
//                        .setTargetPosition(41,-68)
//                        .setMaxPower(.4) // .4 - 6.
//                        .setXyGain(.03)
//                        .setTargetHeading(-94)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, -0.0004))
//                        .setMaxTurnPower(.5)
//                        .setMovementThresh(2)
//                        .setHeadingThreshold(2)
//                        .stopMotorsOnDone(true)
//                        .build())
//                .build();
    }
}
