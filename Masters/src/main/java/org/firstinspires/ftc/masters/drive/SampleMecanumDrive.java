package org.firstinspires.ftc.masters.drive;

import static org.firstinspires.ftc.masters.CSCons.servo1Down;
import static org.firstinspires.ftc.masters.CSCons.servo1Up;
import static org.firstinspires.ftc.masters.CSCons.servo2Down;
import static org.firstinspires.ftc.masters.CSCons.servo2Up;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.masters.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.masters.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.masters.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.masters.drive.DriveConstants.kA;
import static org.firstinspires.ftc.masters.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.masters.drive.DriveConstants.kV;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.masters.PropFindLeftProcessor;
import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.masters.apriltesting.SkystoneDatabase;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.masters.util.LynxModuleUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@Config
public class SampleMecanumDrive extends MecanumDrive {

//    public static double ALIGN_SPEED = .3;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8.6, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.03;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    protected TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private static final boolean USE_WEBCAM = true;

    protected TrajectoryFollower follower;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    //private Encoder leftEncoder, rightEncoder, middleEncoder;
    private List<DcMotorEx> motors;

    DcMotor backSlides = null;
    DcMotor otherBackSlides;
    Servo planeRaise;
    DcMotor intake = null;
    Servo intakeHeight = null;

    public DigitalChannel frontBreakBeam, backBreakBeam;
    public boolean pixelBack, pixelFront;
    Servo outtakeRotation;
    Servo outtakeMovement;
    Servo outtakeMovementRight;
    private Servo wristServo;
    private Servo outtakeServo1, outtakeServo2;
    private Servo transferServo;


    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.05;

    private final double ticks_in_degrees = 384.5 / 180;

//    public final double iticks_in_degree = 384.5 / 180;
//
//    public static double ip = 0.01, ii = 0, iid = 0.00;
//    public static double iif = 0.05;

    protected AprilTagProcessor aprilTag;
    protected PropFindRightProcessor propFindProcessor;
    protected VisionPortal myVisionPortal;
    private WebcamName frontWebcam, backWebcam;
    TelemetryPacket packet = new TelemetryPacket();

    Telemetry telemetry;
    HardwareMap hardwareMap;

    public SampleMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap);
        this.telemetry= telemetry;

    }

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.hardwareMap = hardwareMap;

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);


        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

//        // TODO: adjust the names of the following hardware devices to match your configuration
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
//        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        backSlides = hardwareMap.dcMotor.get("backSlides");
        otherBackSlides = hardwareMap.dcMotor.get("otherBackSlides");

        planeRaise = hardwareMap.servo.get("planeRaise");
        wristServo = hardwareMap.servo.get("wrist");
        //cameraTurning = hardwareMap.servo.get("cameraTurning");

        outtakeServo1 = hardwareMap.servo.get("outtakeHook");
        outtakeServo2 = hardwareMap.servo.get("microHook");

        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovement = hardwareMap.servo.get("backSlideServo");
        outtakeMovementRight = hardwareMap.servo.get("backSlideServoRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeHeight = hardwareMap.servo.get("intakeServo");

        frontBreakBeam = hardwareMap.digitalChannel.get("breakBeam2");
        frontBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        backBreakBeam = hardwareMap.digitalChannel.get("breakBeam1");
        backBreakBeam.setMode(DigitalChannel.Mode.INPUT);

        transferServo = hardwareMap.servo.get("transfer");

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);


        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

        backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otherBackSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        otherBackSlides.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initializeAprilTagProcessing(){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary((SkystoneDatabase.SkystoneDatabase()))
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
    }

    public void initializePropFindRightProcessing(){
        propFindProcessor = new PropFindRightProcessor(telemetry,packet);
    }

    public void initializePropFindLeftProcessing(){
        propFindProcessor = new PropFindLeftProcessor(telemetry,packet);
    }



    public void initializeVisionPortal(){

        frontWebcam = hardwareMap.get(WebcamName.class, "frontWebcam");
        backWebcam = hardwareMap.get(WebcamName.class, "backWebcam");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(frontWebcam, backWebcam);

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(switchableCamera)
                    .setCameraResolution(new Size(640, 360))
                    .addProcessors(propFindProcessor, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(propFindProcessor, aprilTag)
                    .build();
        }


    }

    public VisionPortal getMyVisionPortal() {
        return myVisionPortal;
    }

    public AprilTagProcessor getAprilTag() {
        return aprilTag;
    }

    public PropFindRightProcessor getPropFindProcessor() {
        return propFindProcessor;
    }

    public void activateFrontCamera (){

        myVisionPortal.getCameraState();
        myVisionPortal.setActiveCamera(frontWebcam);
    }

    public  void activateBackCamera(){
        myVisionPortal.setActiveCamera(backWebcam);
    }

    public void enableAprilTag(){
        myVisionPortal.setProcessorEnabled(aprilTag, true);
        myVisionPortal.setProcessorEnabled(propFindProcessor, false);
    }

    public void enablePropProcessor(){
        myVisionPortal.setProcessorEnabled(propFindProcessor, true);
        myVisionPortal.setProcessorEnabled(aprilTag, false);
    }

    public void stopPropProcessor(){
        myVisionPortal.setProcessorEnabled(propFindProcessor, false);
    }

    public void openClaw(){
        ;
    }

    public void transferClaw(){

    }

    public void closeClaw(){

    }

    public void haltSlides() {

    }

    public void startIntake(){
        intake.setPower(CSCons.speed);
    }

    public void revertIntake(){
        intake.setPower(-CSCons.speed);
    }

    public void  stopIntake(){
        intake.setPower(0);
    }

    public void dropIntake(){
        intakeHeight.setPosition(CSCons.intakeGround);
    }

    public void raiseIntake(){
        intakeHeight.setPosition(CSCons.intakeInit);
    }

    public void intakeToGround() {
//        clawAngle.setPosition(CSCons.clawAngleGroundToThree);
//        clawArm.setPosition(CSCons.clawArmGround);
    }

    public void intakeOverStack(){
        intakeHeight.setPosition(CSCons.intakeAboveTop);
    }

    public void intakeToTopStack() {
        intakeHeight.setPosition(CSCons.intake5);
    }

    public void intakeToPosition4(){
        intakeHeight.setPosition(CSCons.intake4);
    }
    //pick up pixel 3 and 4
    public void intakeToPosition3(){
;       intakeHeight.setPosition(CSCons.intake3);
    }

    public void intakeToPosition2(){
        intakeHeight.setPosition(CSCons.intake2);
    }

    public void intakeToPosition1(){
        intakeHeight.setPosition(CSCons.intakeBottom);
    }




    public void intakeToTransfer() {
//        clawAngle.setPosition(CSCons.clawAngleTransfer);
//        clawArm.setPosition(clawArmTransfer);
    }

    public void closeHook(){
//        microHook.setPosition(CSCons.closeMicroHook);
//        outtakeHook.setPosition(CSCons.closeHook);
    }
//     public void closeSmallHook(){
//        microHook.setPosition(CSCons.closeMicroHook);
//     }

//     public void openLargeHook(){
//        outtakeHook.setPosition(openHook);
//     }

     public void openSmallHook(){
//        microHook.setPosition(openMicroHook);
     }

    public void outtakeToBackdrop() {
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementBackdrop);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementBackdrop);
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleBackdrop);
    }

    public void outtakeToTransfer() {
//        outtakeHook.setPosition(CSCons.openHook);
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);
        wristServo.setPosition(CSCons.wristVertical);
    }

    public void outtakeToPickup(){
        outtakeMovement.setPosition(CSCons.wristOuttakePickup);
        outtakeMovementRight.setPosition(CSCons.wristOuttakePickup);
        outtakeRotation.setPosition(CSCons.wristOuttakeAnglePickup);
    }

    public void dropPixel() {
//        outtakeHook.setPosition(CSCons.openHook);
//        microHook.setPosition(CSCons.openMicroHook);
    }

    public void closeFingers(){
        outtakeServo1.setPosition(servo1Down);
        outtakeServo2.setPosition(servo2Down);
    }

    public void closeFrontFingers(){
        outtakeServo1.setPosition(servo1Down);
    }

    public void closeBackFingers(){
        outtakeServo2.setPosition(servo2Down);
    }


    public void openFingers(){
        outtakeServo1.setPosition(servo1Up);
        outtakeServo2.setPosition(servo2Up);
    }

    public void openFrontFinger(){
        outtakeServo1.setPosition(servo1Up);
    }

    public void openBackFinger(){
        outtakeServo2.setPosition(servo2Up);
    }



    @SuppressLint("SuspiciousIndentation")
    public  Pose2d aprilTagCoarsePosEstimate(List<AprilTagDetection> currentDetections) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) { // 72 - 7.5, 72 - 29.25: left blue april tag
                Pose2d aprilTagPos = null;
                if (detection.id == 1) {
                    aprilTagPos = new Pose2d(72 - 7.5, 72 - 29.25, 0);
                } else {
                    aprilTagPos = new Pose2d(0, 0, 0);
                }
                double theta = 90 + Math.abs(detection.ftcPose.bearing) - Math.abs(detection.ftcPose.yaw);
                double xOffset = Math.abs(detection.ftcPose.range * Math.sin(Math.toRadians(theta)));
                double yOffset =  Math.abs(detection.ftcPose.range * Math.cos(Math.toRadians(theta)));

                if (detection.ftcPose.x<0){

                    yOffset= -yOffset;
                }

                if (telemetry!=null) {
                    telemetry.addData("Xoffset", xOffset);
                    telemetry.addData("Yoffset", yOffset);
                }
                Pose2d cameraPosition = null;

                double cameraXOffset = CSCons.cameraOffsetX;
                double cameraYOffset = CSCons.cameraOffsetY;

                switch(detection.id){
                    case 1:

                        cameraPosition = new Pose2d(CSCons.tagBackboardX - xOffset, CSCons.tag1Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                    break;
                    case 2:

                        cameraPosition = new Pose2d(CSCons.tagBackboardX - xOffset, CSCons.tag2Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                        break;
                    case 3:
                        cameraPosition = new Pose2d(CSCons.tagBackboardX - xOffset, CSCons.tag3Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                        break;
                    case 4:
                        cameraPosition =  new Pose2d(CSCons.tagBackboardX - xOffset, CSCons.tag4Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                        break;

                    case 5:
                        cameraPosition =  new Pose2d(CSCons.tagBackboardX - xOffset, CSCons.tag5Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                        break;
                    case 6:
                        cameraPosition =  new Pose2d(CSCons.tagBackboardX - xOffset, CSCons.tag6Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                        break;
                    case 7:
                        cameraPosition =  new Pose2d(CSCons.tagAudienceX + xOffset, CSCons.tag7Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                        cameraXOffset= CSCons.cameraFrontOffsetX;
                        cameraYOffset= CSCons.cameraFrontOffsetY;
                        break;
                    case 8:
                        continue;
                        //cameraPosition =  new Pose2d(CSCons.tagAudienceX + xOffset, CSCons.tag8Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                       // break;
                    case 9:
                        continue;
                        //cameraPosition =  new Pose2d(CSCons.tagAudienceX + xOffset, CSCons.tag9Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                       // break;
                    case 10:
                        cameraXOffset= CSCons.cameraFrontOffsetX;
                        cameraYOffset= CSCons.cameraFrontOffsetY;
                        cameraPosition =  new Pose2d(CSCons.tagAudienceX + xOffset, CSCons.tag10Y + yOffset, Math.toRadians(detection.ftcPose.yaw));
                        break;

                }


                Pose2d robotPosition;
                double xOffsetRobot=0;
                double yOffsetRobot=0;

                if (detection.id==10 || detection.id==7) {
                    xOffsetRobot = Math.abs(cameraXOffset * Math.cos(Math.toRadians(detection.ftcPose.yaw)));
                    yOffsetRobot = cameraYOffset * Math.sin(Math.toRadians(90 + detection.ftcPose.yaw));
                } else {

                    xOffsetRobot = - Math.abs(CSCons.cameraOffsetX * Math.cos(Math.toRadians(180 + detection.ftcPose.yaw)));
                    yOffsetRobot = CSCons.cameraOffsetX * Math.sin(Math.toRadians(180 + detection.ftcPose.yaw));
                }



                if (telemetry!=null){
                    telemetry.addData("robot x offset", xOffsetRobot);
                    telemetry.addData("robot y offset", yOffsetRobot);
                    telemetry.addData("camera x", cameraPosition.getX());
                    telemetry.addData("camera y", cameraPosition.getY());
                    telemetry.addData("tag id", detection.id);
                }
                if (cameraPosition!=null) {

                    if (detection.id==7 || detection.id==10){
                        robotPosition = new Pose2d(cameraPosition.getX() + xOffsetRobot, cameraPosition.getY() + yOffsetRobot, Math.toRadians(180 - detection.ftcPose.yaw));

                    } else
                    robotPosition = new Pose2d(cameraPosition.getX() + xOffsetRobot, cameraPosition.getY() + yOffsetRobot, Math.toRadians(180 + detection.ftcPose.yaw));
                } else {
                    robotPosition = getPoseEstimate();
                }

                return robotPosition;

            }
        }   // end for() loop
        return getPoseEstimate();
    }

//
//    public void alignTag(List<AprilTagDetection> currentDetections, int targetTag) {
//        if (currentDetections != null) {
//            Pose2d relTagPos = getRelativeTagPos(currentDetections,targetTag);
//            if (relTagPos.getX() != 8641000) {
//                while (relTagPos.getHeading() > 4 || relTagPos.getHeading() < -4 ) {
//                    relTagPos = getRelativeTagPos(currentDetections,targetTag);
//                    if (relTagPos.getHeading() > 4) {
//                        setMotorPowers(-.25,-.25,.25,.25);
//                    }
//                    else if (relTagPos.getHeading() < -4) {
//                        setMotorPowers(.25,.25,-.25,-.25);
//                    }
//                }
//            }
//        }
//    }

    public void turnToTag(List<AprilTagDetection> currentDetections, int targetTag) {
        Pose2d relTagPos = getRelativeTagPos(currentDetections,targetTag);
        this.turn(Math.toRadians(relTagPos.getHeading()));
    }

    public void strafeToTag(List<AprilTagDetection> currentDetections, int targetTag) {
        Pose2d relTagPos = getRelativeTagPos(currentDetections,targetTag);
        Trajectory strafe;
        if (relTagPos.getX() < 0) {
            strafe = this.trajectoryBuilder(getPoseEstimate(), false)
                    .strafeRight(relTagPos.getX())
                    .build();
        } else {
            strafe = this.trajectoryBuilder(getPoseEstimate(), false)
                    .strafeLeft(relTagPos.getX())
                    .build();
        }
        this.followTrajectoryAsync(strafe);
    }

    public void fwdToTag(List<AprilTagDetection> currentDetections, int targetTag) {
        Pose2d relTagPos = getRelativeTagPos(currentDetections,targetTag);
        Trajectory fwd;
        if (relTagPos.getY() < 0) {
            fwd = this.trajectoryBuilder(getPoseEstimate(), false)
                    .back(relTagPos.getY()-5)
                    .build();
        } else {
            fwd = this.trajectoryBuilder(getPoseEstimate(), false)
                    .back(relTagPos.getY()-5)
                    .build();
        }
        this.followTrajectoryAsync(fwd);
    }

    public void trajToTag(List<AprilTagDetection> currentDetections, int targetTag) {
        Pose2d relTagPos = getRelativeTagPos(currentDetections,targetTag);
        Trajectory align;
        align = this.trajectoryBuilder(new Pose2d(), false)
                .splineToLinearHeading(new Pose2d(new Vector2d(getPoseEstimate().getX() + relTagPos.getY()-5,getPoseEstimate().getY() + relTagPos.getX()),Math.toRadians(0)),Math.toRadians(0))
                .build();
        this.followTrajectoryAsync(align);
    }

    public Pose2d getRelativeTagPos (List<AprilTagDetection> currentDetections, int targetTag) {
        double tag_x = 8641000;
        double tag_y =0;
        double tag_yaw = 0;
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) { // 72 - 7.5, 72 - 29.25: left blue april tag
                    if (detection.id == targetTag) {
                        tag_x = detection.ftcPose.x;
                        tag_y = detection.ftcPose.y;
                        tag_yaw = detection.ftcPose.yaw;
                    }
                }
            }
        }
        return new Pose2d(new Vector2d(tag_x,tag_y),tag_yaw);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, false, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;//imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) 0.0; //(double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }


    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void breakFollowing() {

    }

    public void backSlidesMove(int target) {

        int slidePos = backSlides.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double liftPower = pid + ff;

        if (telemetry!=null) {
            telemetry.addData("liftPower", liftPower);
        }
        backSlides.setPower(liftPower);
        otherBackSlides.setPower(liftPower);
    }

//    public void intakeSlidesMove(int itarget) {
//
//
//        int islidePos = intakeSlides.getCurrentPosition();
//        double ipid = icontroller.calculate(islidePos, itarget);
//        double iff = Math.cos(Math.toRadians(itarget / iticks_in_degree)) * iif;
//
//        double iliftPower = ipid + iff;
//
//        iliftPower= Math.min(0.8, iliftPower);
//
//        intakeSlides.setPower(iliftPower);
//
//    }

    public DcMotor getBackSlides(){
        return backSlides;
    }

//    public DcMotor getIntakeSlides() {
////        return intakeSlides;
//    }

//    public RevColorSensorV3 getColorSensor(){
//        return colorSensor;
//    }

    public void setWristServoPosition(CSCons.OuttakeWrist outtakeWristPosition){
        if (outtakeWristPosition == CSCons.OuttakeWrist.angleLeft) {
            wristServo.setPosition(CSCons.wristAngleLeft);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.angleRight) {
            wristServo.setPosition(CSCons.wristAngleRight);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.vertical) {
            wristServo.setPosition(CSCons.wristVertical);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.flatLeft) {
            wristServo.setPosition(CSCons.wristFlatLeft);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.flatRight) {
            wristServo.setPosition(CSCons.wristFlatRight);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.verticalDown) {
            wristServo.setPosition(CSCons.wristVerticalDown);
        }
    }

    public void setOuttakeToTransfer(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        wristServo.setPosition(CSCons.wristVertical);
    }

    public void liftOuttake(){
        outtakeMovement.setPosition(0.7);
        outtakeMovementRight.setPosition(0.7);
        outtakeRotation.setPosition(0.8);
    }

    public void setOuttakeToGround(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleBackdrop);
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementGround);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementGround);
    }

    public void pushPixels(){
        transferServo.setPosition(CSCons.transferPush);
    }

    public void raiseTransferArm(){
        transferServo.setPosition(CSCons.transferUp);
    }

    public void initPlane(){
        planeRaise.setPosition(CSCons.droneFlat);
    }




}