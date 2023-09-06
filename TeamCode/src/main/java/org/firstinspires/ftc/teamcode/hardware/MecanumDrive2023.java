package org.firstinspires.ftc.teamcode.hardware;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class MecanumDrive2023 {
    public static class Parameters {
        public double     COUNTS_PER_MOTOR_REV    = 1120 ;   // 1120 per revolution
        public double     DRIVE_GEAR_REDUCTION    = 24.0/32.0; //   3/4
        public double     WHEEL_DIAMETER_INCHES   = 96/25.4 ;     // For figuring circumference
        public double  COUNTS_PER_INCH_FORWARD = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
        public double ROBOT_DIAMETER_IN = 13;
        public double COUNTS_PER_ROTATE = (ROBOT_DIAMETER_IN * Math.PI)*COUNTS_PER_INCH_FORWARD;
        // These constants define the desired driving/control characteristics
        // They can/should be tweaked to suit the specific robot drive train.
        public double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
        public double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
        public double     HEADING_THRESHOLD       = 1 ;    // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
        // Define the Proportional control coefficient (or GAIN) for "heading control".
        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
        // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
        // Decrease these numbers if the heading does not settle on the correct value
        public double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
        //maybe only use one of these.
        public double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

        public int [] _FREE_WHEELS; // no encoder wheels (RIGHT, LEFT)
        public int [] _ENCODER_WHEELS; // encoder wheels (RIGHT, LEFT)
        public int [] _REVERSED_WHEELS; // reversed motors.
        public DcMotor [] motors;
        public boolean robotCentric = true;
        public BNO055IMU imu;
        public Telemetry telemetry;
        public double _SPEED_FACTOR =1.4;
        public double _ROTATION_RATE =0.75;
        public double _INCHES_PER_METER=39.3701;
        public double _ROBOT_INCHES_FRONT=3;
    }

    protected double _ROBOT_INCHES_FRONT;
    protected double _INCHES_PER_METER;
    //protected BNO055IMU imu;
    protected ImuDevice imu;
    protected double     COUNTS_PER_MOTOR_REV;   // 1120 per revolution
    protected double     DRIVE_GEAR_REDUCTION; //   3/4
    protected double     WHEEL_DIAMETER_INCHES;     // For figuring circumference
    protected double     COUNTS_PER_INCH_FORWARD;

    protected double ROBOT_DIAMETER;
    protected double COUNTS_PER_ROTATE;
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    protected double     DRIVE_SPEED;     // Max driving speed for better distance accuracy.
    protected double     TURN_SPEED;     // Max Turn speed to limit turn rate
    protected double     HEADING_THRESHOLD;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value
    protected double     P_TURN_GAIN;     // Larger is more responsive, but also less stable
    //maybe only use one of these.
    protected double     P_DRIVE_GAIN;     // Larger is more responsive, but also less stable

    protected int [] _FREE_WHEELS; // no encoder wheels (RIGHT, LEFT)
    protected int [] _ENCODER_WHEELS; // encoder wheels (RIGHT, LEFT)
    protected int [] _REVERSED_WHEELS; // reversed motors.
    protected DcMotor [] motors;
    protected boolean robotCentric;
    protected double _SPEED_FACTOR;
    protected double _ROTATION_RATE;
    protected Telemetry _telemetry;
    protected Telemetry.Item T_angle;

    public MecanumDrive2023(Parameters parameters) {
        this.COUNTS_PER_MOTOR_REV = parameters.COUNTS_PER_MOTOR_REV;
        this.DRIVE_GEAR_REDUCTION = parameters.DRIVE_GEAR_REDUCTION;
        this.WHEEL_DIAMETER_INCHES = parameters.WHEEL_DIAMETER_INCHES;
        this.COUNTS_PER_INCH_FORWARD = parameters.COUNTS_PER_INCH_FORWARD;
        this.ROBOT_DIAMETER = parameters.ROBOT_DIAMETER_IN;
        this.COUNTS_PER_ROTATE = parameters.COUNTS_PER_ROTATE;
        this.DRIVE_SPEED = parameters.DRIVE_SPEED;
        this.TURN_SPEED = parameters.TURN_SPEED;
        this.HEADING_THRESHOLD = parameters.HEADING_THRESHOLD;
        this.P_TURN_GAIN = parameters.P_TURN_GAIN;
        this.P_DRIVE_GAIN = parameters.P_DRIVE_GAIN;
        this._FREE_WHEELS = parameters._FREE_WHEELS;
        this._ENCODER_WHEELS = parameters._ENCODER_WHEELS;
        this._REVERSED_WHEELS = parameters._REVERSED_WHEELS;
        this._ROTATION_RATE = parameters._ROTATION_RATE;
        this._SPEED_FACTOR = parameters._SPEED_FACTOR;
        this.motors = parameters.motors;
        this.robotCentric = parameters.robotCentric;
        this.imu = new ImuDevice(parameters.imu);
        this._telemetry = parameters.telemetry;
        initAutoMecanum();

    }

    // constants
    protected static final double SQRT2=Math.sqrt(2);
    protected static final double SQRT2_OVER2 = SQRT2/2;
    protected static final double PI_OVER4=Math.PI/4;
    protected  static final double PI_OVER2=Math.PI/2;

    public boolean getRobotCentric() {
        return robotCentric;
    }

    public double getRotationRate() {
        return _ROTATION_RATE;
    }

    public double getSpeedFactor() {
        return _SPEED_FACTOR;
    }

    protected void initAutoMecanum() {
        setRunMode(_FREE_WHEELS, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);

        setDirection(_REVERSED_WHEELS, DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(new int[]{0,1,2,3},DcMotor.ZeroPowerBehavior.BRAKE);


        COUNTS_PER_INCH_FORWARD         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
        T_angle = _telemetry.addData("Heading", imu.getHeading());

    }

    public void movePolar(double power, double angle, double rotate) {
        angle -= PI_OVER4;
        if (! robotCentric ) {
            //make this robot drive in a field centric way.
            angle -= imu.getHeading();
        }
        rotate *= _ROTATION_RATE;
        double sine  = Math.sin(angle);
        double cosine = Math.cos(angle);
        double scale = ( (power + Math.abs(rotate)) > 1 ) ? _SPEED_FACTOR /(power + rotate) : _SPEED_FACTOR /Math.sqrt(2) ;

        double [] wheelSpeeds = {
                scale * (power * sine - rotate),   // Front Right
                scale * (power * cosine - rotate), // Rear Rights
                scale * (power * sine + rotate),   // Rear Left
                scale * (power * cosine + rotate)  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
        T_angle.setValue(imu.getHeading());
    }

    public void moveRect(double forward, double lateral, double rotate) {
        //translate into polar and move accordingly.
        movePolar(Math.hypot(forward,lateral),
                Math.atan2(-forward,lateral),
                rotate);
    }

    public int [] readEncoders() {  // return the encoder values if there are encoder motors.
        if (_ENCODER_WHEELS != null && _ENCODER_WHEELS.length > 0 ) {
            int [] encoders = new int[_ENCODER_WHEELS.length];
            for (int i=0; i<_ENCODER_WHEELS.length; i++) {
                encoders[i] = motors[_ENCODER_WHEELS[i]].getCurrentPosition();
            }
            return encoders;
        }
        return null;
    }

    public void setDirection(int [] wheels, DcMotorSimple.Direction dir) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)
                motors[wheel].setDirection(dir);

    }

    public void setMotorSpeeds(double [] speeds) {
        for (int i=0; i< motors.length; i++) motors[i].setPower(Range.clip(speeds[i], -1, 1));
    }

    public void setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
    }

    public void setRotationRate(double RotationRate) {
        this._ROTATION_RATE = RotationRate;
    }

    public void setRunMode(int [] wheels, DcMotor.RunMode mode) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)
                motors[wheel].setMode(mode);
    }

    public void setSpeedFactor(double SpeedFactor) {
        this._SPEED_FACTOR = SpeedFactor;
    }

    public void setZeroPowerBehavior( int [] wheels, DcMotor.ZeroPowerBehavior behavior) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)
                motors[wheel].setZeroPowerBehavior(behavior);
    }

}
