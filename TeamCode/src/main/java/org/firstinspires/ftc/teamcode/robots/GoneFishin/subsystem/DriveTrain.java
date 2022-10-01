package org.firstinspires.ftc.teamcode.robots.GoneFishin.subsystem;


import static org.firstinspires.ftc.teamcode.robots.GoneFishin.util.Utils.servoNormalize;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.DISTANCE_SENSOR_TO_FRONT_AXLE;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.DISTANCE_TARGET_TO_BACK_WHEEL;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.MAX_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.MIN_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.USE_MOTOR_SMOOTHING;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.robots.GoneFishin.simulation.ServoSim;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.CRServoSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.CloneFollower;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config(value = "FFCrane")
public class DriveTrain implements Subsystem {




    private double targetHeading, targetVelocity = 0;

    private Pose2d poseEstimate, poseError, poseVelocity;
    private Pose2d driveVelocity, lastDriveVelocity;

    private long lastLoopTime, loopTime;


    //devices ---------------------------------------------------------

    private DcMotorEx leftMotor = null;
    private DcMotorEx rightMotor = null;

    private final BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private double compensatedBatteryVoltage;

    //PID LOOPS_______________________________________________________________________

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0);
    public static double HEADING_PID_TOLERANCE = 1;
    public static PIDCoefficients VELOCITY_PID = new PIDCoefficients(4, 0, 0);

    public static PIDController headingPID;
    public static PIDController velocityPID;

    public DriveTrain (HardwareMap hardwareMap){





            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            leftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
            rightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");

            List<DcMotorEx> motors = Arrays.asList(leftMotor, rightMotor, swerveMotor, swivelMotor);

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



                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);




        headingPID = new PIDController(HEADING_PID);
        headingPID.setInputRange(0, Math.toRadians(360));
        headingPID.setOutputRange(-100, 100);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();

        velocityPID = new PIDController(VELOCITY_PID);
        velocityPID.setInputRange(0, Math.toRadians(360));
        velocityPID.setOutputRange(-100, 100);
        velocityPID.setContinuous(true);
        velocityPID.setTolerance(HEADING_PID_TOLERANCE);
        velocityPID.enable();



        driveVelocity = new Pose2d(0, 0, 0);
        lastDriveVelocity = new Pose2d(0, 0, 0);

    }

    @Override
    public void update(Canvas fieldOverlay) {
        double currentHeading = imu.getAngularOrientation();
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}
