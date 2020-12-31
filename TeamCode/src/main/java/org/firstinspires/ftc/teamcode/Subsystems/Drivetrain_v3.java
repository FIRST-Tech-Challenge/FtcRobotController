package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain_v3 {
        // Define hardware objects
        public DcMotor leftFront = null;
        public DcMotor rightFront = null;
        public BNO055IMU imu = null;

        // List constants
        public static final double COUNTS_PER_DRIVE_MOTOR_REV = 28;         // HD Hex encoder counts at the motor (28)
        public static final double ULTRAPLANETARY_REDUCTION = 18.88; //27.4 is for nominal 25:1 18.88 is nominal 20:1 Ultra planetary
        public static final double WHEEL_DIAMETER_INCHES = 3.54;      // 90mm wheels. For figuring circumference its a 90 millimeter wheel
        public static final double COUNTS_PER_INCH = (COUNTS_PER_DRIVE_MOTOR_REV * ULTRAPLANETARY_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        public static final double DRIVE_SPEED = 1;
        private static final double TURN_SPEED = 0.5;
        private boolean inTeleOp;
        private ElapsedTime runtime = new ElapsedTime();


        // Contructor for Drivetrain
        // Passing boolean to automatically config encoders for auto or teleop.
        public Drivetrain_v3(boolean inTeleOp) {
        this.inTeleOp = inTeleOp;
        }

        // initialize drivetrain components, assign names for driver station config, set directions
        // and encoders if needed.
        public void init(HardwareMap hwMap) {

            // initialize the imu first.
            // Note this in NOT IMU calibration.
            imu = hwMap.get(BNO055IMU.class, "imu");



            // initialize al the drive motors
            leftFront = hwMap.get(DcMotor.class, "Left_front");
            rightFront = hwMap.get(DcMotor.class, "Right_front");

            // For HD Planetary Forward yields CCW rotation when shaft is facing you.
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // not in teleop means autonomous so encoders are needed
            // IMU us also needed
            if (!inTeleOp) {
               //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

                //parameters.mode                = BNO055IMU.SensorMode.IMU;
                //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
                //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                //parameters.loggingEnabled      = false;

                // Initialize here calibrate in Opmode
                //imu.initialize(parameters);



                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // FLOAT allows motors to coast when count is reached. Change to BRAKE
                // if necessary
                //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            } else {
                // for InTeleop we don't need encoders because driver controls
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }

        }



}


