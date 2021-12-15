package org.firstinspires.ftc.teamcode.drive;

import android.content.Context;
import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.AdrianControls.VuforiaStuff;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.security.ProtectionDomain;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class MecanumDrive6340 extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);
   public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(35, 0, 7, 14);
    public double currentVoltage;


    public static double LATERAL_MULTIPLIER = 0.93;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    protected DcMotorEx leftFront, leftRear, rightRear, rightFront;
    protected List<DcMotorEx> motors;
    public DcMotorEx intake;
    public DcMotorEx duckMotor;
    public DcMotorEx rotorMotor;
    public DcMotorEx shooter;
    public DcMotorEx ArmMotor;
    public DcMotorEx arm;
    public DcMotorEx indexer;


    protected BNO055IMU imu;

   public DigitalChannel redLED;
   public DigitalChannel greenLED;

    /*
    Instantiate servos
     */
    public Servo leftServo, rightServo, armServo, shooterServo;
    public Servo handServo, elbowServo, boxServo;
    public CRServo leftWheelServo, rightWheelServo;
    public AnalogInput armPOT;
    public DigitalChannel digitalTouch;




    protected VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;
    public int targetEncoderCountLevelMinus1 = 0;
    public int targetEncoderCountLevel0 = 1200;
    public int targetEncoderCountLevel1 = 1200;
    public int targetEncoderCountLevel2 = 2200;
    public int targetEncoderCountLevel3 = 3200;
    public double armMinPowerDuringMove = 1.0;
    public double armMinPowerDuringMoveTeleop = 0.40;
    public double armMinPowerDuringHold = 0.02;

    public MecanumDrive6340(HardwareMap hardwareMap) {        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        Context context = AppUtil.getInstance().getApplication();

        FtcDashboard.start(context);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
      //  arm = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        rotorMotor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        shooter = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        duckMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        indexer = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touchSensor");
        // Get the LED colors and touch sensor from the hardwaremap
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");


        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Turn on RUN_USING_ENCODER
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set PIDF Coefficients with voltage compensated feedforward value
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d,
                SHOOTER_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));


        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        }

        // reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rotorMotor.setDirection(DcMotor.Direction.FORWARD);

        //define and initialize all installed servos
      //  rightServo = hardwareMap.get(Servo.class, "rightServo");
   //     leftServo = hardwareMap.get(Servo.class, "leftServo");
    //   shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        rightWheelServo = hardwareMap.get(CRServo.class, "rightWheelServo");
        leftWheelServo = hardwareMap.get(CRServo.class, "leftWheelServo");
    //   armServo = hardwareMap.get(Servo.class, "armServo");
        handServo = hardwareMap.get(Servo.class, "handServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        boxServo = hardwareMap.get(Servo.class, "boxServo");

        armPOT = hardwareMap.get(AnalogInput.class, "armPOT");
        double currentVoltage = armPOT.getVoltage();

        // if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer((new StandardTrackingWheelLocalizer(hardwareMap) ));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError (deg)", Math.toDegrees(lastError.getHeading()));

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose, getPoseVelocity()));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
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
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
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
        return imu.getAngularOrientation().firstAngle;
    }


    /*
    Ice commands
         */

    public void intakeRings (){
        intake.setPower (1);
        indexer.setPower (-1);

    }
    public void outakeRings () {
        intake.setPower(-1);
        indexer.setPower(1);
    }

    public void armFirstLevel () {
        int startingPosition = ArmMotor.getCurrentPosition();
        ArmMotor.setPower(0.3);
        ArmMotor.setTargetPosition(176);
        while(ArmMotor.isBusy()){

        }
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    //Shooter
   public void shootRings () {
        shooter.setVelocity(1600);
        if (shooter.getVelocity() > 1550) {
            shooterServo.setPosition(0);//LOAD

            shooterServo.setPosition(1);//FIRE
        }
    }

   public void shootPowerShots () {
        shooter.setVelocity(1300);
        if (shooter.getVelocity() > 1250) {
            shooterServo.setPosition(0);//LOAD

            shooterServo.setPosition(1);//FIRE
        }
    }

    //Arm
    public void deployArm() {

          rotorMotor.setPower(0.3);
                }

    public void retractArm(){
        arm.setPower(-0.4);
    }

    public void deliverGoal(){
        if (armPOT.getVoltage() > 1.60) {
            rotorMotor.setPower(-0.5);
               } else  if (armPOT.getVoltage() < 1.0) {
            rotorMotor.setPower(0.3);
        }else rotorMotor.setPower(-0.2);
        }

    //Grab goal
    public void grabGoal (){
        leftServo.setPosition(0.9);
        rightServo.setPosition(0);
        armServo.setPosition(0);

    }
    //Release goal
    public void releaseGoal () {
        leftServo.setPosition(0);
        rightServo.setPosition(1);
        armServo.setPosition(1);

    }

    public void spinwheelright () {
        duckMotor.setPower(1.0);
    }
    public void spinwheelleft () {
        duckMotor.setPower(-1.0);

    }
    public void spinwheel (int Teamcolor) {
        if (Teamcolor == 1.0){
            spinwheelright();
        }
        else{
            spinwheelleft();
        }
    }
    public void spinwheelstop () {
        duckMotor.setPower(0.0);

    }

    public void outTakeblocks (){
        leftWheelServo.setPower(1.0);
        rightWheelServo.setPower(-1.0);
    }
    public void inTakeblocks (){
        leftWheelServo.setPower(-1.0);
        rightWheelServo.setPower(1.0);
         greenLED.setState(false);
         redLED.setState(true);
    }
    public void stopIntakeBlocks(){
        leftWheelServo.setPower(0.0);
        rightWheelServo.setPower(0.0);
        redLED.setState(false);
        greenLED.setState(true);
    }

    public void RotorArmFunctionGo(){
        rotorMotor.setPower(1.0);

    }
    public void RotorArmFunctionBack(){
        rotorMotor.setPower(-1.0);

    }
    public void RotorArmStop(){
        rotorMotor.setPower(0.0);
    }

    public void ArmLifter (int level,int pidTimerInSeconds){
        int targetEnconderCountLevel=targetEncoderCountLevel3;
        if(level ==  3){
            targetEnconderCountLevel = targetEncoderCountLevel3;
        }
        if(level ==  2){
            targetEnconderCountLevel = targetEncoderCountLevel2;
        }
        if(level ==  1){
            targetEnconderCountLevel = targetEncoderCountLevel1;
        }
        if(level ==  0){
            targetEnconderCountLevel = targetEncoderCountLevel0;
        }
        if(level ==  -1){
            targetEnconderCountLevel = targetEncoderCountLevelMinus1;
        }

        ElapsedTime timerForPid = new ElapsedTime();
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int initialPosition = ArmMotor.getCurrentPosition();
        timerForPid.reset();
        double powerToApply = 0.0;
        /*
        if(ArmMotor.getCurrentPosition()-targetEnconderCountLevel<0){
            //ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            powerToApply=armMinPowerDuringMove;
            Log.d("Came Here: ", "Came here: ");
        }
        else{
            //ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            powerToApply=-1.0*armMinPowerDuringMove;
        }

         */
        if(ArmMotor.getCurrentPosition() - targetEnconderCountLevel < -100){
            powerToApply = 0.7;
        }
        else if(ArmMotor.getCurrentPosition() - targetEnconderCountLevel > 100){
            powerToApply = -0.7;
        }
        else{
            powerToApply = 0.0;
            ArmMotor.setPower(powerToApply);
        }
        /*
        if(targetEnconderCountLevel==0){
            powerToApply=0.0;
        }
*/
        while(Math.abs(ArmMotor.getCurrentPosition() - targetEnconderCountLevel)> 100 && timerForPid.seconds()<pidTimerInSeconds)
        {
            /*
            powerToApply = Math.abs(ArmMotor.getCurrentPosition() - targetEnconderCountLevel) * 1.0/Math.abs(initialPosition-targetEnconderCountLevel);
            if (powerToApply<armMinPowerDuringMove){
                powerToApply = armMinPowerDuringMove;
            }
            powerToApply=armMinPowerDuringMove;

             */
            ArmMotor.setPower(powerToApply);
            Log.d("ArmLifter powerToApply: ", String.valueOf(powerToApply));
            Log.d("CurrentPosition ", String.valueOf(ArmMotor.getCurrentPosition()));
            Log.d("Targe Post", String.valueOf(targetEnconderCountLevel));

        }
        ArmMotor.setPower(0.0);
  /*      if(targetEnconderCountLevel==0){
            powerToApply=0.0;
        }
        else
        {
            ArmMotor.setPower(armMinPowerDuringHold);
        }

   */
        //ArmMotor.ZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double ArmLifterPowerToApplyCalculation(int level,int pidTimerInSeconds){
            int targetEnconderCountLevel=targetEncoderCountLevel3;
            if(level ==  3){
                targetEnconderCountLevel = targetEncoderCountLevel3;
            }
            if(level ==  2){
                targetEnconderCountLevel = targetEncoderCountLevel2;
            }
            if(level ==  1){
                targetEnconderCountLevel = targetEncoderCountLevel1;
            }
            if(level ==  0){
                targetEnconderCountLevel = targetEncoderCountLevel0;
            }
            if(level ==  -1){
                targetEnconderCountLevel = targetEncoderCountLevelMinus1;
            }

            int initialPosition = ArmMotor.getCurrentPosition();
            double powerToApply = 0.0;
            if(ArmMotor.getCurrentPosition() > targetEnconderCountLevel+500 || ArmMotor.getCurrentPosition() > targetEnconderCountLevel-500 ){
              //  ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                powerToApply=-1.0;
                Log.d("Came Here: ", "Came here: ");
            }
        if(ArmMotor.getCurrentPosition() < targetEnconderCountLevel+500 || ArmMotor.getCurrentPosition() < targetEnconderCountLevel-500 ){
           // ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            powerToApply=1.0;
            Log.d("Came Here: ", "Came here: ");
        }
          return powerToApply;

    }


}
