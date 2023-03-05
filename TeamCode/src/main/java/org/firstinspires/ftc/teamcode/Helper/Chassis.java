package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.*;

public class Chassis {

    // global location.
    public int robotX = 0;
    public int robotY = 0;

    public int[] Location = {robotX,robotY};



    //IMU
    public  BNO055IMU imu;

    public Orientation angles;

    private ElapsedTime runtime = new ElapsedTime();

    int timeout_ms;



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

    HardwareMap hwMap = null;



    //private static LinearOpmode opModeObj;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;

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

        // Set behavior when zero power is applied.
        FLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Setting the run mode
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        imu                             = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";

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

    public boolean isRobotStable(){
        angles = imu.getAngularOrientation();
        if(Math.abs(angles.secondAngle) >= 5 || Math.abs(angles.thirdAngle) >= 5) {
            return false;
        } else {
            return true;
        }
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

        int FLPos = FLMotor.getCurrentPosition();
        int FRPos = FRMotor.getCurrentPosition();
        int BLPos = BLMotor.getCurrentPosition();
        int BRPos = BRMotor.getCurrentPosition();

        targetFR = FRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBR = BRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetFL = FLPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBL = BLPos + (int) (distance * COUNTS_PER_CM_Hex);

        //Set motor targets
        FLMotor.setTargetPosition(targetFL);
        BLMotor.setTargetPosition(targetBL);
        FRMotor.setTargetPosition(targetFR);
        BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (FLMotor.isBusy() && FRMotor.isBusy())) {

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

        int FLPos = FLMotor.getCurrentPosition();
        int FRPos = FRMotor.getCurrentPosition();
        int BLPos = BLMotor.getCurrentPosition();
        int BRPos = BRMotor.getCurrentPosition();

        targetFR = FRPos - (int) (distance * COUNTS_PER_CM_Hex);
        targetBR = BRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetFL = FLPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBL = BLPos - (int) (distance * COUNTS_PER_CM_Hex);

        //Set motor targets
        FLMotor.setTargetPosition(targetFL);
        BLMotor.setTargetPosition(targetBL);
        FRMotor.setTargetPosition(targetFR);
        BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (FLMotor.isBusy() && FRMotor.isBusy())) {

        }
        this.stopDriveMotors();
    }

    public void stopDriveMotors(){
        // Stop all motion;
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

}
