package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.List;
import java.util.*;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Robot {
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;
    public DcMotor vSlider;
    public DcMotor swingArm;
    public Servo claw;

    private double holdingPower = -0.01;
    public double swingArmHoldingPower = 0.08;

    public int robotX = 0;
    public int robotY = 0;

    public int[] Location = {robotX,robotY};

    //IMU
    public static BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;


    //Drivetrain Motor
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;

    //IMU
    //How many times the encoder counts a tick per revolution of the motor.
    static final double COUNTS_PER_MOTOR_REV_Hex= 538; // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    //Gear ratio of the motor to the wheel. 1:1 would mean that 1 turn of the motor is one turn of the wheel, 2:1 would mean two turns of the motor is one turn of the wheel, and so on.
    static final double DRIVE_GEAR_REDUCTION= 1; // This is < 1.0 if geared UP

    //Diameter of the wheel in CM
    static final double WHEEL_DIAMETER_CM= 10; // For figuring circumference

    //How many times the encoder counts a tick per CM moved. (Ticks per rev * Gear ration) / perimeter
    static final double COUNTS_PER_CM_Hex = (COUNTS_PER_MOTOR_REV_Hex * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_CM * 3.1415);


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


    //private static LinearOpmode opModeObj;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        claw = hwMap.get(Servo.class, "claw");
        vSlider = hwMap.get(DcMotor.class, "vSlider");
        swingArm = hwMap.get(DcMotor.class, "swingArm");

        //Init motors and servos
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");



        //Setting the direction
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);


        claw.setDirection(Servo.Direction.FORWARD);
        vSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        swingArm.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set behavior when zero power is applied.
        FLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vSlider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        swingArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Setting the run mode
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swingArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hwMap.get(BNO055IMU.class, "imu");

         //Since our Rev Expansion is in Vertical Position, so we need to Z & X

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap y and z axes
//        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        TimeUnit.MILLISECONDS.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        TimeUnit.MILLISECONDS.sleep(100); //Changing modes again requires a delay

        imu.initialize(parameters);
    }


    public void Drive(double speed, int distance) {

        runtime.reset();
        timeout_ms = 10000;

        robotY += distance;
        Location[1] = robotY;

        int targetFL;
        int targetFR;
        int targetBR;
        int targetBL;

        int FLPos = this.FLMotor.getCurrentPosition();
        int FRPos = this.FRMotor.getCurrentPosition();
        int BLPos = this.BLMotor.getCurrentPosition();
        int BRPos = this.BRMotor.getCurrentPosition();

        targetFR = FRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBR = BRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetFL = FLPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBL = BLPos + (int) (distance * COUNTS_PER_CM_Hex);

        //Set motor targets
        this.FLMotor.setTargetPosition(targetFL);
        this.BLMotor.setTargetPosition(targetBL);
        this.FRMotor.setTargetPosition(targetFR);
        this.BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        this.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (this.FLMotor.isBusy() && this.FRMotor.isBusy())) {

        }
        this.stopDriveMotors();
    }

    public void Strafe(double speed, int distance) {

        robotX += distance;

        runtime.reset();
        timeout_ms = 10000;

        robotX += distance;
        Location[0] = robotX;

        int targetFL;
        int targetFR;
        int targetBR;
        int targetBL;

        int FLPos = this.FLMotor.getCurrentPosition();
        int FRPos = this.FRMotor.getCurrentPosition();
        int BLPos = this.BLMotor.getCurrentPosition();
        int BRPos = this.BRMotor.getCurrentPosition();

        targetFR = FRPos - (int) (distance * COUNTS_PER_CM_Hex);
        targetBR = BRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetFL = FLPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBL = BLPos - (int) (distance * COUNTS_PER_CM_Hex);

        //Set motor targets
        this.FLMotor.setTargetPosition(targetFL);
        this.BLMotor.setTargetPosition(targetBL);
        this.FRMotor.setTargetPosition(targetFR);
        this.BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        this.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (this.FLMotor.isBusy() && this.FRMotor.isBusy())) {

        }
        this.stopDriveMotors();
    }

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

    public void DriveToPosition(double Speed, int posX, int posY, boolean forwardFirst) {
        if(forwardFirst) {
            this.Drive(Speed, -posY);
            this.Strafe(Speed, -posX);
        }
        else{
            this.Strafe(Speed, -posX);
            this.Drive(Speed, -posY);
        };
        System.out.println(Arrays.toString(Location));
    }

    public float modAngle(float angle) {
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    //Turns the robot
    public void turnRobotToAngle(float endAngle) {
        org.firstinspires.ftc.robotcore.external.navigation.Orientation angle;
        angle = this.imu.getAngularOrientation();

        float angleStart = modAngle(angle.firstAngle);
        float angleEnd = modAngle(endAngle);
        float angleCurrent = angleStart;
        float direction = 0;

        if(modAngle((angleEnd - angleCurrent)) >= 180) {
            //Go CW
            direction = -1;
        } else if (modAngle((angleEnd - angleCurrent)) <= 180) {
            //Go CCW
            direction = 1;
        }

        double pwr = -0.75;


        while (Math.abs(angleCurrent - angleEnd) > 10) {
            FLMotor.setPower(-pwr * direction);
            FRMotor.setPower(pwr * direction);
            BLMotor.setPower(-pwr * direction);
            BRMotor.setPower(pwr * direction);
            angle = this.imu.getAngularOrientation();
            angleCurrent = modAngle(angle.firstAngle);

        }
    }

    public void MoveSlider(double speed, int Position) {
        timeout_ms = 5000;

        runtime.reset();

        this.vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.vSlider.setTargetPosition(Position);

        //set the mode to go to the target position
        this.vSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Set the power of the motor.
        vSlider.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (this.vSlider.isBusy())) {

        }
        this.vSlider.setPower(0);
        this.vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SwingArmToPosition(double speed, int Position) {
        timeout_ms = 3000;

        runtime.reset();

        this.swingArm.setTargetPosition(Position);

        //set the mode to go to the target position
        this.swingArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        swingArm.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (this.swingArm.isBusy())) {

        }
    }

}