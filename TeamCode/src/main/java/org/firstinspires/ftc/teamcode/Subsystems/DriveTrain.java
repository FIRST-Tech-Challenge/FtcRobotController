package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotContainer;

/** DriveTrain Subsystem */
public class DriveTrain extends SubsystemBase {

    // constants for Tetrix DC Motor
    final double MAXRPM = 6000.0;
    final double MAXRPS = MAXRPM/60.0;

    // motor speed ticks per revolution to m/s travel speed
    final double TICKS_PER_ROTATION = (28.0);       // encoder pulses per motor revolution
    final double MAX_SPEED_TICKS_PER_SEC = MAXRPS * TICKS_PER_ROTATION;
    final double WHEEL_DIA = 0.1;                   // wheel diameter in m
    final double GEAR_RATIO = 12.0;                 // drive gear ratio
    final double TICKSPS_TO_MPS = WHEEL_DIA * Math.PI / TICKS_PER_ROTATION / GEAR_RATIO;
    final double MPS_TO_TICKSPS = 1.0 / TICKSPS_TO_MPS;
    public final double MAX_SPEED = MAXRPS * Math.PI * WHEEL_DIA / GEAR_RATIO;

    // Setup Drive Kinematics Object Constants
    final double TRACK_WIDTH = 0.31;   // Width between the left and right wheels - in m.
    final double TRACK_LENGTH = 0.29;  // Length between the front and back wheel - in m.
    private MecanumDriveKinematics driveKinematics;

    // create Mecanum drive and its motors
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;


    /** Place code here to initialize subsystem */
    public DriveTrain() {

        // Initialize the hardware variables. Note that the strings used here must correspond

        // setup the Kinematics
        driveKinematics = new MecanumDriveKinematics(
                // Front Left
                new Translation2d(TRACK_LENGTH * 0.5, TRACK_WIDTH * 0.5),
                // Front Right
                new Translation2d(TRACK_LENGTH * 0.5, TRACK_WIDTH * -0.5),
                // Back Left
                new Translation2d(TRACK_LENGTH * -0.5, TRACK_WIDTH * 0.5),
                // Back Right
                new Translation2d(TRACK_LENGTH * -0.5, TRACK_WIDTH * -0.5));

        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive  = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        // With the shift to DcMotorEx, Inverted function shifted to setDirection.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // set motor speed control PID coefficients
        leftFrontDrive.setVelocityPIDFCoefficients (18.0, 0.0, 0.0, 11.7);
        leftBackDrive.setVelocityPIDFCoefficients (18.0, 0.0, 0.0, 11.7);
        rightFrontDrive.setVelocityPIDFCoefficients (18.0, 0.0, 0.0, 11.7);
        rightBackDrive.setVelocityPIDFCoefficients (18.0, 0.0, 0.0, 11.7);

        // set motor to closed-loop speed control mode
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set initial speeds to zero
        leftFrontDrive.setVelocity(0.0);
        leftBackDrive.setVelocity(0.0);
        rightFrontDrive.setVelocity(0.0);
        rightBackDrive.setVelocity(0.0);

        // set motor braking mode
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        RobotContainer.DBTelemetry.addData("Robot Left Front Speed: ", "%.2f", GetWheelSpeeds().frontLeftMetersPerSecond);
        RobotContainer.DBTelemetry.addData("Robot Left Back Speed: ", "%.2f", GetWheelSpeeds().rearLeftMetersPerSecond);
        RobotContainer.DBTelemetry.addData("Robot Right Front Speed: ", "%.2f", GetWheelSpeeds().frontRightMetersPerSecond);
        RobotContainer.DBTelemetry.addData("Robot Right Back Speed: ", "%.2f", GetWheelSpeeds().rearRightMetersPerSecond);
        RobotContainer.DBTelemetry.update();
    }



    /** drive robot in field coordinates
     * Inputs: X, y and Rotation speed - all -1 to +1 */
    public void FieldDrive (double Vx, double Vy, double Omega){
        FieldDrive(Vx, Vy, Omega, 1.0);
    }

    public void FieldDrive (double Vx, double Vy, double Omega, double powerFactor) {

        // get angle of vector rotation angle
        // i.e. neg of gyro angle - in rad
        double rotAngRad = Math.toRadians(RobotContainer.gyro.getYawAngle());

        // rotate speed vector by negative of gyro angle
        double x = Vx*Math.cos(-rotAngRad) - Vy*Math.sin(-rotAngRad);
        double y = Vx*Math.sin(-rotAngRad) + Vy*Math.cos(-rotAngRad);

        // x,y now in robot coordinates - call robot drive
        RobotDrive(x, y, Omega, powerFactor);
    }

    public void RobotDrive (double Vx, double Vy, double Omega){
        RobotDrive(Vx, Vy, Omega, 1.0);
    }

    /** drive robot in robot coordinates
     * Inputs: X, y and Rotation speed */
    public void RobotDrive (double Vx, double Vy, double Omega, double powerFactor) {
        // create a chassis speed object and populate with x, y, and omega
        ChassisSpeeds driveChassisSpeeds = new ChassisSpeeds(Vx * powerFactor,
                Vy * powerFactor, Omega * powerFactor);

        // determine desired wheel speeds from the chassis speeds
        // rotate around center of robot (i.e. coordinate 0,0)
        MecanumDriveWheelSpeeds WheelSpeeds;
        WheelSpeeds = driveKinematics.toWheelSpeeds(driveChassisSpeeds, new Translation2d(0,0));

        // normalize wheel speeds so no wheel exceeds maximum attainable (in m/s)
        WheelSpeeds.normalize(MAX_SPEED);

        // set individual motor speeds
        double requestedLeftFrontDriveVelocity = WheelSpeeds.frontLeftMetersPerSecond;
        double requestedRightFrontDriveVelocity = WheelSpeeds.frontRightMetersPerSecond;
        double requestedLeftBackDriveVelocity = WheelSpeeds.rearLeftMetersPerSecond;
        double requestedRightBackDriveVelocity = WheelSpeeds.rearRightMetersPerSecond;

        // update telemetry to requested velocities
        RobotContainer.DBTelemetry.addData("Vx Speed: ", "%.2f", Vx * powerFactor);
        RobotContainer.DBTelemetry.addData("Vy Speed: ", "%.2f", Vy * powerFactor);
        RobotContainer.DBTelemetry.addData("Omega: ", "%.2f", Omega);
        RobotContainer.DBTelemetry.addData("Requested Left Front Velocity: ", "%.2f", requestedLeftFrontDriveVelocity);
        RobotContainer.DBTelemetry.addData("Requested Right Front Velocity: ", "%.2f", requestedRightFrontDriveVelocity);
        RobotContainer.DBTelemetry.addData("Requested Left Back Velocity: ", "%.2f", requestedLeftBackDriveVelocity);
        RobotContainer.DBTelemetry.addData("Requested Right Back Velocity: ", "%.2f", requestedRightBackDriveVelocity);
        RobotContainer.DBTelemetry.update();

        // set motor velocities to requested velocities
        leftFrontDrive.setVelocity(MPS_TO_TICKSPS * requestedLeftFrontDriveVelocity);
        rightFrontDrive.setVelocity(MPS_TO_TICKSPS * requestedRightFrontDriveVelocity);
        leftBackDrive.setVelocity(MPS_TO_TICKSPS * requestedLeftBackDriveVelocity);
        rightBackDrive.setVelocity(MPS_TO_TICKSPS * requestedRightBackDriveVelocity);
    }

    /** returns current speeds of mecanum drive wheels in m/s */
    public MecanumDriveWheelSpeeds GetWheelSpeeds()
    {
        // motor speeds are in encoder ticks/s - convert to m/s
        MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds();
        speeds.rearLeftMetersPerSecond = TICKSPS_TO_MPS * leftBackDrive.getVelocity();
        speeds.frontLeftMetersPerSecond = TICKSPS_TO_MPS * leftFrontDrive.getVelocity();
        speeds.rearRightMetersPerSecond = TICKSPS_TO_MPS * rightBackDrive.getVelocity();
        speeds.frontRightMetersPerSecond = TICKSPS_TO_MPS * rightFrontDrive.getVelocity();

        return speeds;
    }

    /* returns the defined mecanum drive kinematics */
    public MecanumDriveKinematics GetKinematics() {
        return driveKinematics;
    }

}
