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
    final double MAXRPM = 5500.0;
    final double MAXRPS = MAXRPM / 60.0;

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
    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;

    // individual motor PIF controls
    private MotorControl leftFrontControl;
    private MotorControl leftBackControl;
    private MotorControl rightFrontControl;
    private MotorControl rightBackControl;

    /**
     * Place code here to initialize subsystem
     */
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
        leftBackDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        // With the shift to DcMotorEx, Inverted function shifted to setDirection.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // set motor to closed-loop speed control mode
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Set initial speeds to zero
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);

        // set motor braking mode
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // create motor PIF controls
        leftFrontControl = new MotorControl();
        leftBackControl = new MotorControl();
        rightFrontControl = new MotorControl();
        rightBackControl = new MotorControl();
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        //RobotContainer.DBTelemetry.addData("Robot Left Front Speed: ", "%.2f", GetWheelSpeeds().frontLeftMetersPerSecond);
        //RobotContainer.DBTelemetry.addData("Robot Left Back Speed: ", "%.2f", GetWheelSpeeds().rearLeftMetersPerSecond);
        //RobotContainer.DBTelemetry.addData("Robot Right Front Speed: ", "%.2f", GetWheelSpeeds().frontRightMetersPerSecond);
        //RobotContainer.DBTelemetry.addData("Robot Right Back Speed: ", "%.2f", GetWheelSpeeds().rearRightMetersPerSecond);
        //RobotContainer.DBTelemetry.update();
    }


    /**
     * drive robot in field coordinates
     * Inputs: X, y and Rotation speed - all -1 to +1
     */
    public void FieldDrive(double Vx, double Vy, double Omega) {
        FieldDrive(Vx, Vy, Omega, 1.0);
    }

    public void FieldDrive(double Vx, double Vy, double Omega, double powerFactor) {

        // get angle of vector rotation angle
        // i.e. neg of gyro angle - in rad
        double rotAngRad = Math.toRadians(RobotContainer.gyro.getYawAngle());

        // rotate speed vector by negative of gyro angle
        double x = Vx * Math.cos(-rotAngRad) - Vy * Math.sin(-rotAngRad);
        double y = Vx * Math.sin(-rotAngRad) + Vy * Math.cos(-rotAngRad);

        // x,y now in robot coordinates - call robot drive
        RobotDrive(x, y, Omega, powerFactor);
    }

    public void RobotDrive(double Vx, double Vy, double Omega) {
        RobotDrive(Vx, Vy, Omega, 1.0);
    }

    /**
     * drive robot in robot coordinates
     * Inputs: X, y and Rotation speed
     */
    public void RobotDrive(double Vx, double Vy, double Omega, double powerFactor) {
        // create a chassis speed object and populate with x, y, and omega
        ChassisSpeeds driveChassisSpeeds = new ChassisSpeeds(Vx * powerFactor,
                Vy * powerFactor, Omega * powerFactor);

        // determine desired wheel speeds from the chassis speeds
        // rotate around center of robot (i.e. coordinate 0,0)
        MecanumDriveWheelSpeeds RefWheelSpeeds;
        RefWheelSpeeds = driveKinematics.toWheelSpeeds(driveChassisSpeeds, new Translation2d(0, 0));

        // normalize wheel speeds so no wheel exceeds maximum attainable (in m/s)
        RefWheelSpeeds.normalize(MAX_SPEED);
        double max = Math.max(Math.abs(RefWheelSpeeds.frontLeftMetersPerSecond), Math.abs(RefWheelSpeeds.frontRightMetersPerSecond));
        max = Math.max(max, Math.abs(RefWheelSpeeds.rearRightMetersPerSecond));
        max = Math.max(max, Math.abs(RefWheelSpeeds.rearLeftMetersPerSecond));
        if (max > MAX_SPEED) {
            double factor = MAX_SPEED/max;
            RefWheelSpeeds.frontLeftMetersPerSecond *= factor;
            RefWheelSpeeds.frontRightMetersPerSecond *= factor;
            RefWheelSpeeds.rearRightMetersPerSecond *= factor;
            RefWheelSpeeds.rearLeftMetersPerSecond *= factor;
        }

        // update telemetry to requested velocities
        //RobotContainer.DBTelemetry.addData("Vx Speed: ", "%.2f", Vx * powerFactor);
        //RobotContainer.DBTelemetry.addData("Vy Speed: ", "%.2f", Vy * powerFactor);
        //RobotContainer.DBTelemetry.addData("Omega: ", "%.2f", Omega);
        //RobotContainer.DBTelemetry.addData("Requested Left Front Velocity: ", "%.2f", RefWheelSpeeds.frontLeftMetersPerSecond);
        //RobotContainer.DBTelemetry.addData("Requested Right Front Velocity: ", "%.2f", RefWheelSpeeds.rearRightMetersPerSecond);
        //RobotContainer.DBTelemetry.addData("Requested Left Back Velocity: ", "%.2f", RefWheelSpeeds.rearLeftMetersPerSecond);
        //RobotContainer.DBTelemetry.addData("Requested Right Back Velocity: ", "%.2f", RefWheelSpeeds.rearRightMetersPerSecond);
        //RobotContainer.DBTelemetry.update();

        // desired wheel RPM from MPS
        double MPStoRPM = GEAR_RATIO * 60.0 / (WHEEL_DIA * Math.PI);
        double RefLeftFrontRPM = MPStoRPM * RefWheelSpeeds.frontLeftMetersPerSecond;
        double RefRightFrontRPM = MPStoRPM * RefWheelSpeeds.frontRightMetersPerSecond;
        double RefLeftBackRPM = MPStoRPM * RefWheelSpeeds.rearLeftMetersPerSecond;
        double RefRightBackRPM = MPStoRPM * RefWheelSpeeds.rearRightMetersPerSecond;

        // current motor speeds in rpm
        double TickstoRPM = 60.0/TICKS_PER_ROTATION;
        double CurrentLeftFrontRPM = TickstoRPM * leftFrontDrive.getVelocity();
        double CurrentRightFrontRPM = TickstoRPM * rightFrontDrive.getVelocity();
        double CurrentLeftBackRPM = TickstoRPM * leftBackDrive.getVelocity();
        double CurrentRightBackRPM = TickstoRPM * rightBackDrive.getVelocity();

        // calculate PIDs and set powers of each motor
        leftFrontDrive.setPower(leftFrontControl.calculate(RefLeftFrontRPM,CurrentLeftFrontRPM));
        rightFrontDrive.setPower(rightFrontControl.calculate(RefRightFrontRPM,CurrentRightFrontRPM));
        leftBackDrive.setPower(leftBackControl.calculate(RefLeftBackRPM,CurrentLeftBackRPM));
        rightBackDrive.setPower(rightBackControl.calculate(RefRightBackRPM, CurrentRightBackRPM));

    }

    // use to stop motor drive - resets controllers
    public void StopDrive() {

        // reset all motor controllers
        leftFrontControl.reset();
        rightFrontControl.reset();
        leftBackControl.reset();
        rightBackControl.reset();

        // stop drive motors
        RobotDrive(0.0, 0.0, 0.0);
    }


    /**
     * returns current speeds of mecanum drive wheels in m/s
     */
    public MecanumDriveWheelSpeeds GetWheelSpeeds() {
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

    // Special motor control class - specifically tailored for drive motor control
    // implemented as subclass to facilitate application to all four drive motors
    // Motor controller class
    private class MotorControl {
        // motor max no-load speed = 6000rpm
        // motor full power control = 1
        // open loop control/rpm = 1.0/6000.0 = 0.00016667
        private double Pgain = 1.5 * 0.00016667;  // was 1.5
        private double Igain = 0.50 * 0.00016667;
        private double Fgain = 1.30 * 0.00016667;  // was 1.2 // use 90% to assume battery voltage typ. at 13V and not 12.


        // Integrated Error
        private double IntegratedError;

        private MotorControl() {
            // reset the motor controller
            reset();
        }

        // resets the motor controller
        public void reset() {
            // reset integrated error
            IntegratedError = 0.0;
        }


        // run the motor control, return desired control action
        private double calculate(double ReferenceSpeed, double CurrentSpeed) {
            // current error in speed
            double error = ReferenceSpeed - CurrentSpeed;

            /////////
            // I controller with non-linear gain
            double ierror = error;
            //if (ierror > 250.0) ierror = 250.0;
            //if (ierror < -250.0) ierror = -250.0;
            if (Math.abs(error) < 250)
                IntegratedError += Igain * ierror;
            else
                IntegratedError += Igain * 0.15 * ierror;

            // anti-windup logic
            if (error > 400.0 && IntegratedError < 0.0)
                IntegratedError = 0.0;
            if (error < -400.0 && IntegratedError > 0.0)
                IntegratedError = 0.0;

            // if integrated error is over-compensating, then help reduce over time
            if (error < 0.0 && IntegratedError > 0.0)
                IntegratedError *= 0.98;
            if (error > 0.0 && IntegratedError < 0.0)
                IntegratedError *= 0.98;

            ///// end I controller special logic

            // implement PIF controller and return the desired control action
            return (Fgain * ReferenceSpeed) + (Pgain * error) + (IntegratedError);
        }
    }

}
