package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;


import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.*;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.wrapAngleRad;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.diffInchesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.swerveInchesToEncoderTicks;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.robots.taubot.Field;
import org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832;
import org.firstinspires.ftc.teamcode.robots.taubot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.taubot.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robots.taubot.util.DiffyKinematics;
import org.firstinspires.ftc.teamcode.robots.taubot.util.PathLine;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Utils;
import org.firstinspires.ftc.teamcode.robots.taubot.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.robots.taubot.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.robots.taubot.util.CloneFollower;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Vector2;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config(value = "PPDriveTrain")
public class DriveTrain extends DiffyDrive implements Subsystem {
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VELOCITY, MAX_ANG_VEL);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCELERATION);
    private double targetHeading, targetVelocity = 0;

    public Pose2d currentPose;
    private Pose2d driveVelocity, lastDriveVelocity;

    private double leftPosition, rightPosition, leftRelOffset, rightRelOffset, swervePosition, swivelPosition;

    private double leftVelocity, rightVelocity;
    private double targetLeftVelocity, targetRightVelocity;
    private double leftPower, rightPower;
    private boolean useMotorPowers;
    private boolean maintainHeadingEnabled, imuOffsetsInitialized;
    private double maintainHeading, maintainHeadingCorrection;
    private double heading, roll, pitch, pitchVelocity, angularVelocity;
    private double headingOffset, rollOffset, pitchOffset;
    public final TrajectorySequenceRunner trajectorySequenceRunner;
    private Pose2d poseEstimate, poseError, poseVelocity;
    private long lastLoopTime, loopTime;

    private static Vector2 position;

    //devices ---------------------------------------------------------
    List<DcMotorEx> motors;
    public DcMotorEx leftMotor = null;
    public DcMotorEx rightMotor = null;

    private BNO055IMU imu = null;
    private VoltageSensor batteryVoltageSensor;

    private double compensatedBatteryVoltage;

    //PID LOOPS_______________________________________________________________________

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1.5, 0, 0);
    public static double HEADING_PID_TOLERANCE = 1;
    public static PIDCoefficients DIST_TRAVELLED_PID = new PIDCoefficients(5, 0.0, 0); //todo tune this - copied from Reach
    public static PIDCoefficients VELOCITY_PID = new PIDCoefficients(4, 0, 0);
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(4, 0, 0);
    public static PIDCoefficients CROSS_AXIAL_PID = new PIDCoefficients(0.001, 0, 0);

    public static PIDController headingPID, distTravelledPID;
    public static PIDController velocityPID;

    private final boolean simulated;


    //grid drive ---------------------------------------------------------------------

    private Stage gridDrive = new Stage();
    private StateMachine driveToNextTarget = Utils.getStateMachine(gridDrive)
            .addState(() -> {return true;})
            .build();
    private PathLine gridPathLine;

    public DriveTrain (HardwareMap hardwareMap, boolean simulated){

        super(simulated);
        useMotorPowers = false;

        this.simulated = simulated;
        TrajectoryFollower follower = new CloneFollower(AXIAL_PID, CROSS_AXIAL_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5)), 1.5);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        if (simulated) {

            leftMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            rightMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            motors = Arrays.asList(leftMotor, rightMotor);
        } else {

            leftMotor = hardwareMap.get(DcMotorEx.class, "motorLeft");
            rightMotor = hardwareMap.get(DcMotorEx.class, "motorRight");
            motors = Arrays.asList(leftMotor, rightMotor);
        }
            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);

                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                compensatedBatteryVoltage = batteryVoltageSensor.getVoltage();
            }

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                imu = hardwareMap.get(BNO055IMU.class, "baseIMU");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
                //because the Expansion hub is upsidedown:
                BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        headingPID = new PIDController(HEADING_PID);
        headingPID.setInputRange(0, Math.toRadians(360));
        headingPID.setOutputRange(-100, 100);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();

        //oof currently this will be in inches units
        //input is in inches, output is drive speed
        distTravelledPID = new PIDController(DIST_TRAVELLED_PID);
        distTravelledPID.setInputRange(-144, 144);
        distTravelledPID.setOutputRange(-30, 30); //todo - what is the Max speed for tombot?
        distTravelledPID.setContinuous(false);
        distTravelledPID.setTolerance(1);
        distTravelledPID.enable();

        velocityPID = new PIDController(VELOCITY_PID);
        velocityPID.setInputRange(0, Math.toRadians(360));
        velocityPID.setOutputRange(-100, 100);
        velocityPID.setContinuous(true);
        velocityPID.setTolerance(HEADING_PID_TOLERANCE);
        velocityPID.enable();                                                                                     

        driveVelocity = new Pose2d(0, 0, 0);
        lastDriveVelocity = new Pose2d(0, 0, 0);

        //default pose - gotta have some initial pose

        driveToNextTarget = Utils.getStateMachine(gridDrive)
                .addState(() -> false)
                .build();

    }

    public void setTargetVelocity(double vel){
        velocityPID.setSetpoint(vel);
    }

    public void setTargetHeading(double vel){
        headingPID.setSetpoint(vel);
        targetHeading = vel;
    }

    public void setTargetHeadingDeg(double vel){
        setTargetHeading(Math.toRadians(vel));
    }

    double rawHeading;

    @Override
    public void update(Canvas fieldOverlay) {

        // sensor reading
        currentStateMachine.execute();

        leftVelocity = diffEncoderTicksToInches(leftMotor.getVelocity());
        rightVelocity = diffEncoderTicksToInches(rightMotor.getVelocity());

        if (simulated) {
            double dt = loopTime / 1e9;
            leftPosition += leftVelocity * dt;
            rightPosition += rightVelocity * dt;

        } else {
            leftPosition = diffEncoderTicksToInches(leftMotor.getCurrentPosition() - leftRelOffset);
            rightPosition = diffEncoderTicksToInches(rightMotor.getCurrentPosition() - rightRelOffset);
        }

        Orientation orientation = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        if (!imuOffsetsInitialized && imu.isGyroCalibrated()) {
            headingOffset = orientation.firstAngle;
            rollOffset = wrapAngleRad(orientation.secondAngle);
            pitchOffset = wrapAngleRad(orientation.thirdAngle);

            imuOffsetsInitialized = true;
        }

        rawHeading = orientation.firstAngle;

        heading = orientation.firstAngle - headingOffset;

        roll = orientation.secondAngle - rollOffset;
        pitch = orientation.thirdAngle - pitchOffset;

        AngularVelocity angularVelocities = imu.getAngularVelocity();
        pitchVelocity = angularVelocities.yRotationRate;
        angularVelocity = angularVelocities.xRotationRate;

        updatePoseEstimate();
        poseEstimate = getPoseEstimate();
        poseVelocity = getPoseVelocity();

        if(trajectorySequenceRunner.isBusy()) {
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity(), fieldOverlay);
            if (signal != null)
                setDriveSignal(signal);
            poseError = trajectorySequenceRunner.getLastPoseError();
        }

        //-------------------------------- actual driving ---------------------------------------

        if(!manualDriveEnabled) {
            driveToNextTarget.execute();
        }

        if (useMotorPowers) {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
        } else {
            leftMotor.setVelocity(diffInchesToEncoderTicks(targetLeftVelocity));
            rightMotor.setVelocity(diffInchesToEncoderTicks(targetRightVelocity));
        }

    }

    public void updatePositionForNextRun(){
        position = new Vector2(getPoseEstimate().getX(),getPoseEstimate().getY());
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("turnStuff", turnAngle - poseEstimate.getHeading());

        if (debug) {
            telemetryMap.put("Grid Drive Index", gridDriveIndex);
            telemetryMap.put("Target Heading", targetHeading);
            telemetryMap.put("Heading", heading);
            telemetryMap.put("x", poseEstimate.getX());
            telemetryMap.put("y", poseEstimate.getY());
            telemetryMap.put("x target", driveTargetPos.getX());
            telemetryMap.put("y target", driveTargetPos.getY());
            telemetryMap.put("pose heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetryMap.put("raw heading", Math.toDegrees(rawHeading));
            telemetryMap.put("raw heading radians", rawHeading);
            telemetryMap.put("heading", Math.toDegrees(heading));

            telemetryMap.put("x vel", poseVelocity.getX());
            telemetryMap.put("y vel", poseVelocity.getY());
            telemetryMap.put("heading vel", Math.toDegrees(poseVelocity.getHeading()));

            if (trajectorySequenceRunner.isBusy()) {
                telemetryMap.put("xError", poseError.getX());
                telemetryMap.put("yError", poseError.getY());
                telemetryMap.put("headingError", Math.toDegrees(poseError.getHeading()));
            }

            telemetryMap.put("roll", Math.toDegrees(roll));
            telemetryMap.put("pitch", Math.toDegrees(pitch));

            telemetryMap.put("left position", leftPosition);
            telemetryMap.put("right position", rightPosition);
            telemetryMap.put("left position tics", diffInchesToEncoderTicks(leftPosition));
            telemetryMap.put("right position tics", diffInchesToEncoderTicks(rightPosition));
            telemetryMap.put("swerve position", swerveInchesToEncoderTicks(swervePosition));

            telemetryMap.put("left velocity", leftVelocity);
            telemetryMap.put("right velocity", rightVelocity);

            telemetryMap.put("target left velocity", targetLeftVelocity);
            telemetryMap.put("target right velocity", targetRightVelocity);

            telemetryMap.put("maintain heading enabled", maintainHeadingEnabled);
            telemetryMap.put("maintain heading", Math.toDegrees(maintainHeading));
            //telemetryMap.put("maintain heading PID on target", maintainHeadingOnTarget);
            telemetryMap.put("maintain heading PID correction", maintainHeadingCorrection);

            telemetryMap.put("angular velocity", Math.toDegrees(angularVelocity));
            telemetryMap.put("pitch velocity", Math.toDegrees(pitchVelocity));

            telemetryMap.put("drive velocity", driveVelocity.toString());
            telemetryMap.put("last drive velocity", lastDriveVelocity.toString());

            telemetryMap.put("left motor power", leftPower);
            telemetryMap.put("right motor power", rightPower);
            telemetryMap.put("usePower", useMotorPowers);
            telemetryMap.put("manualDrive", manualDriveEnabled);

            telemetryMap.put("loop time", loopTime / 1e9);

            if (!simulated) {
                PIDFCoefficients velocityCoefficients = leftMotor
                        .getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetryMap.put("measured drivetrain PID coeffs", String.format("(p: %f, i: %f, d: %f)",
                        velocityCoefficients.p, velocityCoefficients.i, velocityCoefficients.d));
            }
        }

        return telemetryMap;
    }

    public void resetGridDrive(Position start){
        if(PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.AUTONOMOUS)) {
            setPoseEstimate(new Pose2d(start.getPose().getX(), start.getPose().getY()));
            for (DcMotorEx motor : motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }else if (PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.DEMO)){
            setPoseEstimate(new Pose2d(start.getPose().getX(), start.getPose().getY()));
        }else if(PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.TELE_OP)){
            setPoseEstimate(new Pose2d(position.x,position.y));
        }
    }

    @Override
    public String getTelemetryName() {
        return "Drive Train";
    }

    //----------------------------------------------------------------------------
    // Grid Drive
    //---------------------------------------------------------------------------------------------

    public void setGridDriveStateMachine(StateMachine s){
        driveToNextTarget = s;
    }
    public boolean setPath(PathLine path){
        gridPathLine = path;
        followPathInitialized = false;
        return true;
    }
    boolean followPathInitialized = false;
    double startTime = 0;

    double timeStep = 0.1;
    public boolean followPath(){
        if(!followPathInitialized){
            followPathInitialized = true;
            startTime = System.nanoTime()/1e9;
            return false;
        }
        double time = System.nanoTime() / 1e9 + timeStep - startTime;
        Pose2d newPoint = Field.coordinatesToPose(gridPathLine.getPointAtTime(time));
        Pose2d currentPoseEstimate = getPoseEstimate();
        double dx = newPoint.getX() - currentPoseEstimate.getX();
        double dy = newPoint.getY() - currentPoseEstimate.getY();
        double velocity = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2))/timeStep;
        //direction
        double heading = getExternalHeading();

        //todo check this heading code
        double sign = Math.signum(dx * ( Math.cos(heading)) - dy * (Math.sin(heading)));
        double newHeading = wrapAngleRad(Math.atan2(dy,dx) + (sign>=0 ? 0:(Math.PI)));

        headingPID.setSetpoint(newHeading);
        headingPID.setInput(heading);
        double correction = headingPID.performPID();

        //check if the correction is in the right direction
        //todo
        setLeftVelocity(sign*velocity + correction);
        setRightVelocity(sign*velocity-correction);

        return time > gridPathLine.getTotalTime();
    }


    // ----------------------------------------------------------------------------------------------
    // Manual Driving
    // ----------------------------------------------------------------------------------------------
    private boolean manualDriveEnabled = false;
    public void ManualDriveOff(){
        if (manualDriveEnabled){
            manualDriveEnabled=false;
            setMotorPowers(0,0); //stop drive motors
        }
    }

    public void ManualTankDrive(double speedLeft, double speedRight){
        //todo add logic about overriding any behaviors in progress
        manualDriveEnabled = true;
        setMotorPowers(speedLeft,speedRight);
    }

    public void ManualArcadeDrive(double speedForward, double speedTurn){
        //todo add logic about overriding any behaviors in progress
        manualDriveEnabled = true;
        double drive, turn, left, right, max;

        drive = speedForward/3;
        turn  =  speedTurn/4;

        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        setMotorPowers(left,right);
    }

    // ----------------------------------------------------------------------------------------------
    // Trajectory Following
    // ----------------------------------------------------------------------------------------------

    private double getAveragePos() {
        return (leftPosition + rightPosition) / 2.0;
    }

    //reset the relative position to zeros
    private void resetRelPos(){
        leftRelOffset = leftMotor.getCurrentPosition();
        rightRelOffset = rightMotor.getCurrentPosition();
        leftPosition = 0;
        rightPosition = 0;
    }

    boolean withinError(double value, double target, double offset){
        return (value >= target-offset && value <= target+offset);
    }

    Pose2d driveTargetPos = new Pose2d(0,0);

    double getMagnitude(Pose2d pos, Pose2d tar){
        return Math.sqrt(  Math.pow(pos.getX()-tar.getX() , 2) + Math.pow(pos.getY()-tar.getY() , 2)   );
    }

    private double driveTarget, driveHeading, driveSpeed, driveDistErr, driveDistErrPrev;
    private Stage driveStage = new Stage();
    private StateMachine drive = Utils.getStateMachine(driveStage)
            .addState(() -> {
                headingPID.setSetpoint(driveHeading);
                headingPID.setInput(poseEstimate.getHeading());
                double correction = headingPID.performPID();
                //driveDistErr=Math.abs(driveTarget - getAveragePos());
                distTravelledPID.setSetpoint(driveTarget);
                distTravelledPID.setInput(getAveragePos());
                double spd = distTravelledPID.performPID();
                setDriveSignal(new DriveSignal(new Pose2d(spd, 0, correction), new Pose2d(0, 0, 0)));
                return (distTravelledPID.onTarget());
            })
            .build();

    boolean driveUntilInitialized = false;

    //call this version if we just want to continue in the direction the robot is currently pointing
    public boolean driveUntil(double driveDistance, double driveSpeed) {
        return(driveUntil(driveDistance, poseEstimate.getHeading(), driveSpeed));
    }

    public boolean driveUntil(double driveDistance, double driveHeading, double driveSpeed) {
        if(!driveUntilInitialized) {
            //resetRelPos();
            this.driveTarget = driveDistance + getAveragePos();
            this.driveHeading = driveHeading;
            this.driveSpeed = driveSpeed;
            distTravelledPID.setOutputRange(-driveSpeed,driveSpeed); //max speed
            distTravelledPID.setSetpoint(driveTarget);
            driveUntilInitialized = true;
        }

        if(drive.execute()){
            setDriveSignal(new DriveSignal(new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)));

            driveUntilInitialized = false;
            return true;
        }
        return false;
    }
    //version using a heading requested in degrees
    public boolean driveUntilDegrees(double driveDistance, double driveHeading, double driveSpeed) {
        return driveUntil(driveDistance, Math.toRadians(wrapAngle(driveHeading)), driveSpeed);
    }

    //driveAsyncInitialized is only true when its currently driving
    boolean isDriving(){return driveUntilInitialized;}

    private double turnError = 2.0;
    private double turnAngle;
    private Stage turnStage = new Stage();
    private StateMachine turn = Utils.getStateMachine(turnStage)
            .addState(() -> {
                headingPID.setSetpoint(turnAngle);
                headingPID.setInput(poseEstimate.getHeading());
                double correction = headingPID.performPID();
                setDriveSignal(new DriveSignal(new Pose2d(0, 0, correction), new Pose2d(0, 0, 0)));
                return headingPID.onTarget();
            })
            .build();

    boolean turnInit = false;

    public boolean setTurn(double angles){
        setTargetHeading(Math.toRadians(angles));
        if(heading-Math.toRadians(angles) >= -0.05 && heading-Math.toRadians(angles) <= 0.05){
            return true;
        }else{
            return false;
        }
    }

    public boolean turnUntil(double turnAngle) {
        if(!turnInit){
            this.turnAngle = turnAngle; // this is in Radians relative to the starting angle as set by auton alliance
            turnInit = true;
        }

        if(turn.execute()){
            setDriveSignal(new DriveSignal(new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)));
            turnInit = false;
            return true;
        }
        return false;
    }

    //request a turn in degrees units
    public boolean turnUntilDegrees(double turnAngle) {
        return turnUntil(Math.toRadians(wrapAngle(turnAngle)));
    }

    //see isDriving();
    boolean isTurning(){return turnInit;}

    @Override
    public void setDriveSignal(@NonNull DriveSignal driveSignal) {
        useMotorPowers = false;
        List<Double> velocities = DiffyKinematics.robotToWheelVelocities(driveSignal.getVel(), TRACK_WIDTH);

        setMotorVelocities(velocities.get(0), velocities.get(1));

        driveVelocity = driveSignal.getVel();
    }

    public void setDriveVelocity(@NonNull Pose2d driveVelocity) {
        useMotorPowers = false;
        this.driveVelocity = driveVelocity;
        List<Double> velocities = DiffyKinematics.robotToWheelVelocities(driveVelocity, TRACK_WIDTH);
        setMotorVelocities(velocities.get(0), velocities.get(1));
    }

    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        useMotorPowers = true;
        List<Double> powers = DiffyKinematics.robotToWheelVelocities(drivePower, TRACK_WIDTH);
        setMotorPowers(powers.get(0), powers.get(1));
    }

    private static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new TankVelocityConstraint(maxVel, TRACK_WIDTH)));
    }

    private static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL);
    }

    int gridDriveIndex = 0;

    public double getTargetHeading(){
        return targetHeading;
    }

    boolean gridDriveActive = false;

    int gridTargetX;
    int gridTargetY;

    public void incXTarget(){
        if(gridTargetX < 5)gridTargetX++;
    }
    public void decXTarget(){
        if(gridTargetX > 0)gridTargetX--;
    }

    public void incYTarget(){
        if(gridTargetY < 5)gridTargetY++;
    }
    public void decXYarget(){
        if(gridTargetY > 0)gridTargetY--;
    }

    public boolean DriveTo(double x, double y, double speed, boolean xFirst){
        double robotX = poseEstimate.getX();
        double robotY = poseEstimate.getY();

        if(xFirst) {
            switch (gridDriveIndex) {
                case 0:
                    setTargetHeadingDeg(0);
                    if(Utils.withinErrorPercent(heading,targetHeading,0.01)){
                        gridDriveIndex++;
                    }
                    break;
                case 1:
                    if(driveUntilDegrees(x-robotX,0,speed)){
                        gridDriveIndex++;
                    }
                    break;
                case 2:
                    setTargetHeadingDeg(-90);
                    if(Utils.withinErrorPercent(heading,targetHeading,0.01)){
                        gridDriveIndex++;
                    }
                    break;
                case 3:
                    if(driveUntilDegrees(y-robotY,-90,speed)){
                        gridDriveIndex++;
                    }
                    break;
                case 4:
                    gridDriveIndex = 0;
                    return true;
                default:
                    gridDriveIndex = 0;
                    return false;

            }
        }else{
            switch (gridDriveIndex) {
                case 0:
                    setTargetHeadingDeg(-90);
                    if(Utils.withinErrorPercent(heading,targetHeading,0.01)){
                        gridDriveIndex++;
                    }
                    break;
                case 1:
                    if(driveUntilDegrees(y-robotY,-90,speed)){
                        gridDriveIndex++;
                    }
                    break;
                case 2:
                    setTargetHeadingDeg(0);
                    if(Utils.withinErrorPercent(heading,targetHeading,0.01)){
                        gridDriveIndex++;
                    }
                    break;
                case 3:
                    if(driveUntilDegrees(x-robotX,0,speed)){
                        gridDriveIndex++;
                    }
                    break;
                case 4:
                    gridDriveIndex = 0;
                    return true;
                default:
                    return false;

            }
        }
        return false;
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    // ----------------------------------------------------------------------------------------------
    // Getters And Setters
    // ----------------------------------------------------------------------------------------------

    public void setLeftVelocity(double left) {
        this.targetLeftVelocity = left;
    }

    public void setRightVelocity(double right) {
        this.targetRightVelocity = right;
    }


    public void setMotorVelocities(double left, double right) {
        useMotorPowers = false;
        this.targetLeftVelocity = left;
        this.targetRightVelocity = right;
    }

    public void setMotorPowers(double left, double right) {
        useMotorPowers = true;
        this.leftPower = left;
        this.rightPower = right;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void driveIMU(double heading, double velocity){

    }



    @Override
    public Double getExternalHeadingVelocity() {
        return angularVelocity;
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                leftPosition,
                rightPosition);
    }

    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                leftVelocity,
                rightVelocity);
    }

    StateMachine currentStateMachine = Utils.getStateMachine(new Stage()).addState(()->{return true;}).build();

    public Articulation articulate(Articulation target, Position startingPosition){
        Articulation articulation = target;
        switch(articulation){
            case  leftAuton:
                currentStateMachine = Utils.getStateMachine(new Stage())
                        .addState(() -> driveUntilDegrees(2*Field.INCHES_PER_GRID,0,20))
                        .addState(() ->
                                ((startingPosition == Position.START_LEFT )? turnUntilDegrees(-90):turnUntilDegrees(90))
                        )
                        .addState(() ->
                                ((startingPosition == Position.START_LEFT )? driveUntilDegrees(Field.INCHES_PER_GRID,-90,20):driveUntilDegrees(-Field.INCHES_PER_GRID,90,20))
                        )
                        .addState(() -> {return true;})
                        .build();;
                break;
            case middleAuton:
                currentStateMachine = Utils.getStateMachine(new Stage())
                        .addState(() -> driveUntilDegrees(2*Field.INCHES_PER_GRID,0,20))
                        .addState(() ->
                            ((startingPosition == Position.START_LEFT )? turnUntilDegrees(-90):turnUntilDegrees(90))
                        )
                        .addState(() -> {return true;})
                        .build();
                break;
            case rightAuton:
                currentStateMachine = Utils.getStateMachine(new Stage())
                        .addState(() -> driveUntilDegrees(2*Field.INCHES_PER_GRID,0,20))
                        .addState(() ->
                                ((startingPosition == Position.START_LEFT )? turnUntilDegrees(-90):turnUntilDegrees(90))
                        )
                        .addState(() ->
                                ((startingPosition == Position.START_LEFT )? driveUntilDegrees(-Field.INCHES_PER_GRID,-90,20):driveUntilDegrees(Field.INCHES_PER_GRID,90,20))
                        )
                        .addState(() -> {return true;})
                        .build();;
                break;
            default:
                break;

        }
        return target;
    }

    public enum Articulation{
        leftAuton,
        middleAuton,
        rightAuton
    }

    public double getVoltage() {
        return compensatedBatteryVoltage;
    }

    public void setMaintainHeadingEnabled(boolean maintainHeadingEnabled) {
        this.maintainHeadingEnabled = maintainHeadingEnabled;
    }

    public boolean isMaintainHeadingEnabled() {
        return maintainHeadingEnabled;
    }

    public void setMaintainHeading(double maintainHeading) {
        this.maintainHeading = maintainHeading;
    }

    public double getMaintainHeading() {
        return maintainHeading;
    }


    @Override
    protected double getRawExternalHeading() {
        return heading-headingOffset;
    }

    public double getRawHeading(){
        return rawHeading;
    }
}
