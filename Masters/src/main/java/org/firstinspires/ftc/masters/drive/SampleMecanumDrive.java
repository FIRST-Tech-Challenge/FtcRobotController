package org.firstinspires.ftc.masters.drive;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BOTTOM;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.masters.BadgerConstants.CLAW_OPEN;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_MIDDLE;
import static org.firstinspires.ftc.masters.BadgerConstants.TIP_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.TIP_CENTER;
import static org.firstinspires.ftc.masters.BadgerConstants.TIP_FRONT;
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

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.masters.PowerPlayComputerVisionPipelines;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.masters.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {

    public static double ALIGN_SPEED = .35;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 1);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(9, 0, 1);

    public static double LATERAL_MULTIPLIER = 60.0/64.0;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    protected TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    protected TrajectoryFollower follower;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    //private Encoder leftEncoder, rightEncoder, middleEncoder;
    private List<DcMotorEx> motors;
    public DcMotorEx linearSlide;
    public DcMotorEx frontSlide;
    public DcMotorEx slideOtherer;
    public DcMotorEx armMotor;

    private Servo claw, tippingServo;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);


        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
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


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        claw = hardwareMap.servo.get("clawServo");
        tippingServo = hardwareMap.servo.get("tippingServo");


        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }


        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        frontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide"); // This one is named Jim in spirit
        slideOtherer = hardwareMap.get(DcMotorEx.class, "slideOtherer");

        armMotor = hardwareMap.get(DcMotorEx.class, "armServo");


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

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOtherer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideOtherer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideOtherer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void openClaw(){
        claw.setPosition(CLAW_OPEN);
    }

    public void closeClaw(){
        claw.setPosition(CLAW_CLOSED);
    }

    public void tipCenter(){
        tippingServo.setPosition(TIP_CENTER);
    }

    public void tipFront(){
        tippingServo.setPosition(TIP_FRONT);
    }

    public void tipBack(){
        tippingServo.setPosition(TIP_BACK);
    }

    public void liftTop(){
        linearSlide.setTargetPosition(SLIDE_HIGH);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontSlide.setTargetPosition(SLIDE_HIGH);
        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideOtherer.setTargetPosition(SLIDE_HIGH);
        slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.6);
        frontSlide.setPower(.6);
        slideOtherer.setPower(.6);
        //set arm to middle
    }

    public void liftMiddle() {
        closeClaw();
        linearSlide.setTargetPosition(SLIDE_MIDDLE);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontSlide.setTargetPosition(SLIDE_MIDDLE);
        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideOtherer.setTargetPosition(SLIDE_MIDDLE);
        slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.2);
        frontSlide.setPower(.2);
        slideOtherer.setPower(.2);
    }

    public void liftDown() {
        closeClaw();
        linearSlide.setTargetPosition(SLIDE_BOTTOM);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontSlide.setTargetPosition(SLIDE_BOTTOM);
        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideOtherer.setTargetPosition(SLIDE_BOTTOM);
        slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.4);
        frontSlide.setPower(.4);
        slideOtherer.setPower(.4);
    }

    public void setArmServoTop() {
        armMotor.setTargetPosition(ARM_MID_TOP);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
    }

    public void setArmServoBottom() {
        armMotor.setTargetPosition(ARM_BOTTOM);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
    }

    public void setArmServoMiddle() {
        armMotor.setTargetPosition(ARM_MID_TOP);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
    }


    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
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
     //   return 0;

        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().xRotationRate;
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

    public boolean alignPole(PowerPlayComputerVisionPipelines.PipePosition pos) {
        switch (pos) {

            case LEFT3:
            case LEFT4:
            case LEFT5:
            case LEFT6:
            case LEFT7:
            case LEFT8:
                leftFront.setPower(-ALIGN_SPEED);
                leftRear.setPower(-ALIGN_SPEED);
                rightFront.setPower(ALIGN_SPEED);
                rightRear.setPower(ALIGN_SPEED);
                break;

            case RIGHT3:
            case RIGHT4:
            case RIGHT5:
            case RIGHT6:
            case RIGHT7:
            case RIGHT8:
                leftFront.setPower(ALIGN_SPEED);
                leftRear.setPower(ALIGN_SPEED);
                rightFront.setPower(-ALIGN_SPEED);
                rightRear.setPower(-ALIGN_SPEED);
                break;
            case LEFT1:
            case LEFT2:
            case RIGHT1:
            case RIGHT2:
            case CENTER:
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                return true;

        }
        return false;
    }

}