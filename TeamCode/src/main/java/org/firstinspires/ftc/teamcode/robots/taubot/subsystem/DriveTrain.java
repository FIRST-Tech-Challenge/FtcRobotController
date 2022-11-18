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

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.robots.taubot.Field;
import org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832;
import org.firstinspires.ftc.teamcode.robots.taubot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.taubot.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Constants;
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

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config(value = "PPDriveTrain")
public class DriveTrain extends DiffyDrive implements Subsystem {

    private double targetHeading, targetVelocity = 0;


    private Pose2d driveVelocity, lastDriveVelocity;

    private double leftPosition, rightPosition, leftRelOffset, rightRelOffset;

    private double leftVelocity, rightVelocity;
    private double targetLeftVelocity, targetRightVelocity;
    private double leftPower, rightPower;


    private boolean isManualDriveEnabled;
    private double heading, roll, pitch, pitchVelocity, angularVelocity;
    private double headingOffset, rollOffset, pitchOffset;

    private Pose2d poseEstimate, poseError, poseVelocity;
    private long lastLoopTime, loopTime;

    public boolean imuOffsetsInitialized;



    //devices ---------------------------------------------------------
    List<DcMotorEx> motors;
    public DcMotorEx leftMotor = null;
    public DcMotorEx rightMotor = null;

    private BNO055IMU imu = null;
    private VoltageSensor batteryVoltageSensor;

    private double compensatedBatteryVoltage;

    //PID LOOPS_______________________________________________________________________

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.5, 0, 0);
    public static double HEADING_PID_TOLERANCE = 1;


    public static PIDController headingPID;


    private final boolean simulated;



    public DriveTrain (HardwareMap hardwareMap, boolean simulated){

        super(simulated);


        this.simulated = simulated;

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

                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


        driveVelocity = new Pose2d(0, 0, 0);
        lastDriveVelocity = new Pose2d(0, 0, 0);


        setPoseEstimate(Position.START_RIGHT.getPose());



    }




    double rawHeading;

    @Override
    public void update(Canvas fieldOverlay) {



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






        if (isManualDriveEnabled) {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

        } else {
            headingPID.setSetpoint(targetHeading);
            headingPID.setInput(Math.toDegrees(poseEstimate.getHeading()));
            double correction = headingPID.performPID();
            leftMotor.setVelocity(diffInchesToEncoderTicks(targetVelocity + correction));
            rightMotor.setVelocity(diffInchesToEncoderTicks(targetVelocity - correction));
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();


        if (debug) {
            telemetryMap.put("x", poseEstimate.getX());
            telemetryMap.put("y", poseEstimate.getY());
            telemetryMap.put("pose heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetryMap.put("raw heading", Math.toDegrees(rawHeading));
            telemetryMap.put("raw heading radians", rawHeading);
            telemetryMap.put("heading", Math.toDegrees(heading));

            telemetryMap.put("x vel", poseVelocity.getX());
            telemetryMap.put("y vel", poseVelocity.getY());
            telemetryMap.put("heading vel", Math.toDegrees(poseVelocity.getHeading()));



            telemetryMap.put("roll", Math.toDegrees(roll));
            telemetryMap.put("pitch", Math.toDegrees(pitch));

            telemetryMap.put("left position", leftPosition);
            telemetryMap.put("right position", rightPosition);
            telemetryMap.put("left position tics", diffInchesToEncoderTicks(leftPosition));
            telemetryMap.put("right position tics", diffInchesToEncoderTicks(rightPosition));


            telemetryMap.put("left velocity", leftVelocity);
            telemetryMap.put("right velocity", rightVelocity);

            telemetryMap.put("target left velocity", targetLeftVelocity);
            telemetryMap.put("target right velocity", targetRightVelocity);



            telemetryMap.put("angular velocity", Math.toDegrees(angularVelocity));
            telemetryMap.put("pitch velocity", Math.toDegrees(pitchVelocity));

            telemetryMap.put("drive velocity", driveVelocity.toString());
            telemetryMap.put("last drive velocity", lastDriveVelocity.toString());

            telemetryMap.put("left motor power", leftPower);
            telemetryMap.put("right motor power", rightPower);

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

    @Override
    public String getTelemetryName() {
        return "Drive Train";
    }



    // ----------------------------------------------------------------------------------------------
    // Manual Driving
    // ----------------------------------------------------------------------------------------------

    public void ManualDriveOff(){
        isManualDriveEnabled = false;

    }

    public void ManualTankDrive(double speedLeft, double speedRight){
        //todo add logic about overriding any behaviors in progress
        isManualDriveEnabled = true;
        setMotorPowers(speedLeft,speedRight);
    }

    public void ManualArcadeDrive(double speedForward, double speedTurn){
        //todo add logic about overriding any behaviors in progress
        isManualDriveEnabled = true;
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





    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public void setMotorPowers(double left, double right) {

        this.leftPower = left;
        this.rightPower = right;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
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

    @Override
    public void setMotorVelocities(double left, double right) {

    }


    public double getVoltage() {
        return compensatedBatteryVoltage;
    }



    @Override
    protected double getRawExternalHeading() {
        return heading-headingOffset;
    }

    public double getRawHeading(){
        return rawHeading;
    }

    @Override
    public void setDrivePower(@NonNull Pose2d pose2d) {

    }

    @Override
    public void setDriveSignal(@NonNull DriveSignal driveSignal) {

    }
}
