package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Robot {
    private ElapsedTime runtime = new ElapsedTime();

    //Init Motors
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;

    //Init other mechanisms
    public DcMotor intake = null;
    public DcMotor pulley = null;
    public DcMotor arm = null;
    public CRServo carousel = null;
    public Servo boxcover;
    public Servo capping;
    public CRServo sIntake;
    public Servo claw;

    //IMU
    public static BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;

    //How many times the encoder counts a tick per revolution of the motor.
    static final double COUNTS_PER_MOTOR_REV    = 538;      // eg: GoBuilda 5203 Planetery

    //Gear ratio of the motor to the wheel. 1:1 would mean that 1 turn of the motor is one turn of the wheel, 2:1 would mean two turns of the motor is one turn of the wheel, and so on.
    static final double DRIVE_GEAR_REDUCTION    = 1;        // This is < 1.0 if geared UP

    //Diameter of the wheel in CM
    static final double WHEEL_DIAMETER_CM   = 10;     // For figuring circumference

    //How many times the encoder counts a tick per CM moved. (Ticks per rev * Gear ration) / perimeter
    static final double COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);



    /* local OpMode members. */

    //Init hardware map
    HardwareMap hwMap = null;
    HardwareMap imuHwMap = null;

    public ElapsedTime period = new ElapsedTime();
    //tells you how long the robot has run for
    public ElapsedTime test_run_time = new ElapsedTime();
    //this is how you create an instance in a java class ^


    //
    public void Robot() {

    }

    public enum MoveStep {
        yaxix, xaxis, turn, stop
    }

    public MoveStep moveStep;

    //private static LinearOpmode opModeObj;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");

        //controller b
        intake = hwMap.get(DcMotor.class, "intake");
        pulley = hwMap.get(DcMotor.class, "pulley");
        arm = hwMap.get(DcMotor.class, "arm");
        carousel = hwMap.get(CRServo.class, "carousel");

        boxcover = hwMap.get(Servo.class, "boxcover");
        capping = hwMap.get(Servo.class, "capping");
        sIntake = hwMap.get(CRServo.class, "sIntake");
        claw = hwMap.get(Servo.class, "claw");



        //Setting the run mode
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Setting the direction
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        pulley.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(CRServo.Direction.FORWARD);

        boxcover.setDirection(Servo.Direction.FORWARD);
        capping.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        //Stop and reset encoder values.
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Brake the motors and servos
        FLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pulley.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hwMap.get(BNO055IMU.class, "imu");

        // Since our Rev Expansion is in Vertical Position, so we need to Z & X

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        TimeUnit.MILLISECONDS.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        TimeUnit.MILLISECONDS.sleep(100); //Changing modes again requires a delay

        imu.initialize(parameters);

    }

    //Init IMU
    public void IMUinit(HardwareMap ahwMap) throws InterruptedException {
        imuHwMap = ahwMap;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = imuHwMap.get(BNO055IMU.class, "imu");

        // Since our Rev Expansion is in Vertical Position, so we need to Z & X

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        TimeUnit.MILLISECONDS.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        TimeUnit.MILLISECONDS.sleep(100); //Changing modes again requires a delay

        imu.initialize(parameters);
    }


    //Accurately move Forward/Backward
    public void startDriveToPosition(double speed, double distance) {
        int newTargetL;
        int newTargetR;
        int avgLeft = (this.FLMotor.getCurrentPosition() + this.BLMotor.getCurrentPosition()) / 2;
        int avgRight = (this.FRMotor.getCurrentPosition() + this.BRMotor.getCurrentPosition()) / 2;

        newTargetL = avgLeft + (int) (distance * COUNTS_PER_CM);
        newTargetR = avgRight + (int) (distance * COUNTS_PER_CM);
        this.FLMotor.setTargetPosition(newTargetL);
        this.FRMotor.setTargetPosition(newTargetR);
        this.BLMotor.setTargetPosition(newTargetL);
        this.BRMotor.setTargetPosition(newTargetR);


        // Turn On RUN_TO_POSITION
        this.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the start motion.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

    }

    //Stop the motors for drivetrain
    public void stopDriveMotors(){
        // Stop all motion;
        this.FLMotor.setPower(0);
        this.FRMotor.setPower(0);
        this.BLMotor.setPower(0);
        this.BRMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        this.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //Accurately move left/right
    public void startStrafeToPosition(double speed, int distance) {
        int newTargetF;
        int newTargetB;
        int avgFront = (this.FLMotor.getCurrentPosition() + this.BRMotor.getCurrentPosition()) / 2;
        int avgBack = (this.BLMotor.getCurrentPosition() + this.FRMotor.getCurrentPosition()) / 2;
        newTargetF = avgFront + (int) (distance * COUNTS_PER_CM);
        newTargetB = avgBack - (int) (distance * COUNTS_PER_CM);
        this.FLMotor.setTargetPosition(newTargetF);
        this.FRMotor.setTargetPosition(newTargetB);
        this.BLMotor.setTargetPosition(newTargetB);
        this.BRMotor.setTargetPosition(newTargetF);

        // Turn On RUN_TO_POSITION
        this.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

    }

    //Turns a wheel which we used to turn the carousel
    public void turn_carousel(int turns, double pwr, int time_ms) {

        for (int i = 0; i < turns; i++) {
            //reset time
            this.runtime.reset();
            //run for 1 sec
            while(this.runtime.milliseconds() <= time_ms){
                carousel.setPower(pwr);
                //sets power
            }
            //stops power
            carousel.setPower(0);
            break;

        }
    }

    //Stops the robot completely.
    public void robotStop() {
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //Moves the arm to a specific target
    public void moveArmToTarget(int barcode, double speed) {
        int timeout_ms = 1500;
        int targetPosition=0;
        int currentPosition = this.arm.getCurrentPosition();
        if (barcode == 1) { // arm to the front
            this.arm.setTargetPosition(-30);
        }
        else if (barcode == 2) { // arm in the middle
            this.arm.setTargetPosition(-65);
        }
        else if (barcode == 3) { // arm at the back
            this.arm.setTargetPosition(-120);
        }
        // Turn on RUN_TO_POSITION
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion
        runtime.reset();
        this.arm.setPower(Math.abs(speed));

        while ((runtime.milliseconds() < timeout_ms) && (this.arm.isBusy())) {

        }
    }

    //Moves the pulley to a specific target.
    public void movePulleyToTarget(int height, double speed) {
        runtime.reset();
        int timeout_ms = 1000;
        int currentPosition = this.pulley.getCurrentPosition();

        //level 3 height
        if (height == 3) {
            this.pulley.setTargetPosition(1100);
        }
        //level 2 height
        else if (height == 2) {
            this.pulley.setTargetPosition(400);
        }

        //level 1 height
        else if (height == 1) {
            this.pulley.setTargetPosition(0);
        }
        //for shared hub
        else if (height == 4) {
            this.pulley.setTargetPosition(500);
        }

        // Turn on RUN_TO_POSITION
        this.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion
        this.pulley.setPower(Math.abs(speed));

        while ((runtime.milliseconds() < timeout_ms) && (this.pulley.isBusy())) {
        }
    }


    float modAngle(float angle) {
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    //Turns the robot
    public void turnRobot(float turnAngle) {
        org.firstinspires.ftc.robotcore.external.navigation.Orientation angle;
        angle = Robot.imu.getAngularOrientation();

        float angleStart = modAngle(angle.firstAngle);
        float angleEnd = modAngle(angleStart + turnAngle);
        float angleCurrent = angleStart;
        float direction = Math.signum(turnAngle);

        double pwr = 0.3;

        while (Math.abs(angleCurrent - angleEnd) > 1) {
            FLMotor.setPower(-pwr * direction);
            FRMotor.setPower(pwr * direction);
            BLMotor.setPower(-pwr * direction);
            BRMotor.setPower(pwr * direction);
            angle = Robot.imu.getAngularOrientation();
            angleCurrent = modAngle(angle.firstAngle);

        }
    }

    // This will align to the with reference to the angle with that robot started
    // Ideally the start positon is either 360 or 0 -
    //

    //Aligns the robot to be straight.
    public void alignStraight(double startAngle, double finalAngle, double power) {
        double rotation_direction = Math.signum(startAngle-finalAngle);
        double pwr =  power * rotation_direction;

        while (Math.abs(startAngle - finalAngle) > 1) {
            FLMotor.setPower(-pwr);
            FRMotor.setPower(pwr);
            BLMotor.setPower(-pwr);
            BRMotor.setPower(pwr);

            startAngle = getRobotAngle();
        }

    }

    public float getRobotAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle < 0) angles.firstAngle = angles.firstAngle * (-1.0f);
        else angles.firstAngle = -angles.firstAngle + 360.0f;
        return (angles.firstAngle);
    }


    public float getRobotAngleRad() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle);
    }




    private double curveLinearJoystick(double input) {
        return (input / 1.07) * ((0.62 * Math.pow(input, 2)) + 0.45);
    }
    public void fieldCentricMove(double x, double y, double turn) {

        x *= -1.0;
        double power = Math.hypot(x, y);
        double theta = Math.atan2(y, x) - getRobotAngleRad();

        double rx = (Math.sin(theta + (Math.PI / 4))) * power;
        double lx = (Math.sin(theta - (Math.PI / 4))) * power;

        double fl = lx - turn;
        double fr = rx + turn;
        double bl = rx - turn;
        double br = lx + turn;

        setPowers(fl, fr, bl, br);

    }



    public void setPowers(double fl, double fr, double bl, double br) {
        this.FLMotor.setPower(fl);
        this.FRMotor.setPower(fr);
        this.BLMotor.setPower(bl);
        this.BRMotor.setPower(br);
    }

    public void setPowers(double[] powers) {
        setPowers(powers[0], powers[1], powers[2], powers[3]);
    }
}