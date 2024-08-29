package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotContainer;

/** DriveTrain Subsystem */
public class DriveTrain extends SubsystemBase {

    // constants for Tetrix DC Motor
    //final double MAXRPM = 100.0;
    //final double MAXRPS = MAXRPM/60.0;
    //final double MAX_SPEED_TICKS_PER_SEC = MAXRPS * TICKS_PER_ROTATION;

    // motor speed ticks per revolution to m/s travel speed
    final double TICKS_PER_ROTATION = 24.0;        // encoder pulses per motor revolution
    final double WHEEL_DIA = 0.1;                   // wheel diameter in m
    final double GEAR_RATIO = 12.0;                 // drive gear ratio
    final double TICKSPS_TO_MPS = WHEEL_DIA * Math.PI / TICKS_PER_ROTATION / GEAR_RATIO;


    // create Mecanum drive and its motors
    private MotorEx leftFrontDrive = null;
    private MotorEx leftBackDrive = null;
    private MotorEx rightFrontDrive = null;
    private MotorEx rightBackDrive = null;


    /** Place code here to initialize subsystem */
    public DriveTrain() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = new MotorEx(RobotContainer.ActiveOpMode.hardwareMap,"leftFrontDrive");
        leftBackDrive  = new MotorEx(RobotContainer.ActiveOpMode.hardwareMap,"leftBackDrive");
        rightFrontDrive = new MotorEx(RobotContainer.ActiveOpMode.hardwareMap,"rightFrontDrive");
        rightBackDrive = new MotorEx(RobotContainer.ActiveOpMode.hardwareMap,"rightBackDrive");

        //
        leftFrontDrive.setInverted(false);
        leftBackDrive.setInverted(false);
        rightFrontDrive.setInverted(true);
        rightBackDrive.setInverted(true);

        // set motor speed control PID coefficients
        //leftFrontDrive.setVeloCoefficients(0.0, 0.0, 0.0);
        //leftBackDrive.setVeloCoefficients(0.0, 0.0, 0.0);
        //rightFrontDrive.setVeloCoefficients(0.0, 0.0, 0.0);
        //rightBackDrive.setVeloCoefficients(0.0, 0.0, 0.0);

        // set motor speed feed forward coefficients
        //leftFrontDrive.setFeedforwardCoefficients(0.0, 0.025);
        //leftBackDrive.setFeedforwardCoefficients(0.0, 0.025);
        //rightBackDrive.setFeedforwardCoefficients(0.0, 0.025);
        //leftFrontDrive.setFeedforwardCoefficients(0.0, 0.025);

        // set motor to closed-loop speed control mode
        //leftFrontDrive.setRunMode(MotorEx.RunMode.VelocityControl);
        //leftBackDrive.setRunMode(MotorEx.RunMode.VelocityControl);
        //rightFrontDrive.setRunMode(MotorEx.RunMode.VelocityControl);
        //rightBackDrive.setRunMode(MotorEx.RunMode.VelocityControl);

        // set motor braking mode
        leftFrontDrive.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        // distance per pulse in m
        // pi*d / pulse-per-revolution = 3.1415 *0.1m / 28 / 60 (gear ratio)
        final double DistancePerPulse = Math.PI * 0.1 / 28.0 / 60.0;
        leftFrontDrive.setDistancePerPulse(DistancePerPulse);
        leftBackDrive.setDistancePerPulse(DistancePerPulse);
        rightFrontDrive.setDistancePerPulse(DistancePerPulse);
        rightBackDrive.setDistancePerPulse(DistancePerPulse);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
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
        double x = Vx*Math.cos(rotAngRad) - Vy*Math.sin(rotAngRad);
        double y = Vx*Math.sin(rotAngRad) + Vy*Math.cos(rotAngRad);

        // x,y now in robot coordinates - call robot drive
        RobotDrive(x, y, Omega, powerFactor);
    }

    public void RobotDrive (double Vx, double Vy, double Omega){
        RobotDrive(Vx, Vy, Omega, 1.0);
    }

    /** drive robot in robot coordinates
     * Inputs: X, y and Rotation speed */
    public void RobotDrive (double Vx, double Vy, double Omega, double powerFactor) {
        // resolve individual mecanum speeds
        double leftFrontPower  = Vx + Vy + Omega;
        double rightFrontPower = Vx - Vy - Omega;
        double leftBackPower   = Vx - Vy + Omega;
        double rightBackPower  = Vx + Vy - Omega;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // set individual motor speeds
        //leftFrontDrive.setVelocity(MAX_SPEED_TICKS_PER_SEC*leftFrontPower);
        //rightFrontDrive.setVelocity(MAX_SPEED_TICKS_PER_SEC*rightFrontPower);
        //leftBackDrive.setVelocity(MAX_SPEED_TICKS_PER_SEC*leftBackPower);
        //rightBackDrive.setVelocity(MAX_SPEED_TICKS_PER_SEC*rightBackPower);
        leftFrontDrive.set(leftFrontPower * powerFactor);
        rightFrontDrive.set(rightFrontPower * powerFactor);
        leftBackDrive.set(leftBackPower * powerFactor);
        rightBackDrive.set(rightBackPower * powerFactor);
    }

    /** returns current speeds of mecanum drive wheels in m/s */
    public MecanumDriveWheelSpeeds GetWheelSpeeds()
    {
        // motor speeds are in encoder ticks/s - convert to m/s
        MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds();
        speeds.rearLeftMetersPerSecond = 0.7*TICKSPS_TO_MPS*leftBackDrive.getCorrectedVelocity();
        speeds.frontLeftMetersPerSecond = 0.7*TICKSPS_TO_MPS*leftFrontDrive.getCorrectedVelocity();
        speeds.rearRightMetersPerSecond = 0.7*TICKSPS_TO_MPS*rightBackDrive.getCorrectedVelocity();
        speeds.frontRightMetersPerSecond = 0.7*TICKSPS_TO_MPS*rightFrontDrive.getCorrectedVelocity();

        return speeds;
    }

}
