package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static android.os.SystemClock.sleep;

public class Robot {

    public Robot() {}

    final static int TICKS_PER_INCH = 33;

    final static double DEG90 = 22;//90 degree turn distance ()

    static DcMotor rightfront;
    static DcMotor leftfront;
    static DcMotor leftback;
    static DcMotor rightback;
    static DcMotor launcher1;
    static DcMotor launcher2;
    static DcMotor launcherbelt;
    static DcMotor wobbleArmMotor;

    static CRServo feeder;
    static Servo wobbleClaw;
    static Servo wobbleArm;
    static Servo loader;
    static Servo blocker;
    static BNO055IMU imu;
    static Orientation lastAngles = new Orientation();
    static double globalAngle, correction;



    public static void initMotors(OpMode opMode) {
        rightfront = opMode.hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = opMode.hardwareMap.get(DcMotor.class, "leftfront");
        leftback = opMode.hardwareMap.get(DcMotor.class, "leftback");
        rightback = opMode.hardwareMap.get(DcMotor.class, "rightback");

        launcher1 = opMode.hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = opMode.hardwareMap.get(DcMotor.class, "launcher2");
        launcherbelt = opMode.hardwareMap.get(DcMotor.class, "launcher_belt");

        feeder = opMode.hardwareMap.get(CRServo.class, "feeder");
        wobbleClaw = opMode.hardwareMap.servo.get("wobbleClaw");
        wobbleArm = opMode.hardwareMap.servo.get("wobbleArm");
        wobbleArmMotor = opMode.hardwareMap.get(DcMotor.class, "wobbleArmMotor");
        blocker = opMode.hardwareMap.servo.get("blocker");



        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        launcherbelt.setDirection(DcMotor.Direction.FORWARD);
        wobbleArmMotor.setDirection((DcMotor.Direction.REVERSE));
        resetMotors();
    }

    public static void initAccessories(OpMode opMode){
        launcher1 = opMode.hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = opMode.hardwareMap.get(DcMotor.class, "launcher2");
        launcherbelt = opMode.hardwareMap.get(DcMotor.class, "launcher_belt");

        feeder = opMode.hardwareMap.get(CRServo.class, "feeder");
        wobbleClaw = opMode.hardwareMap.servo.get("wobbleClaw");
        wobbleArm = opMode.hardwareMap.servo.get("wobbleArm");
        wobbleArmMotor = opMode.hardwareMap.get(DcMotor.class, "wobbleArmMotor");
        blocker = opMode.hardwareMap.servo.get("blocker");

        launcherbelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherbelt.setDirection(DcMotor.Direction.FORWARD);
        wobbleArmMotor.setDirection((DcMotor.Direction.REVERSE));
    }

    public static void resetMotors() {
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherbelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherbelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void forward(double dist, double speed) {
        move(speed, dist, dist, dist, dist);
    }
    public static void back(double dist, double speed) {
        move(speed, -dist, -dist, -dist, -dist);
    }
    public static void counter(double dist, double speed) {
        move(speed, -dist, -dist, dist, dist);
    }
    public static void clock(double dist, double speed){ move(speed, dist, dist, -dist, -dist); }
    public static void right(double dist, double speed) { move(speed, -dist, dist, dist, -dist); }
    public static void left(double dist, double speed) { move(speed, dist, -dist, -dist, dist); }


    public static void resetEncoders(){
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static double[] encoderValues(){
        double[] encoderArray = {rightfront.getCurrentPosition(), rightback.getCurrentPosition(), leftfront.getCurrentPosition(), leftback.getCurrentPosition()};
        return encoderArray;
    }

    public static void move(double speed, double distRF, double distRB, double distLF, double distLB) {
        resetEncoders();

        int posRF = (int)(distRF * TICKS_PER_INCH);
        int posRB = (int)(distRB * TICKS_PER_INCH);
        int posLF = (int)(distLF * TICKS_PER_INCH);
        int posLB = (int)(distLB * TICKS_PER_INCH);


        rightfront.setTargetPosition(posRF);
        rightback.setTargetPosition(posRB);
        leftfront.setTargetPosition(posLF);
        leftback.setTargetPosition(posLB);

        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightfront.setPower(speed);
        rightback.setPower(speed);
        leftfront.setPower(speed);
        leftback.setPower(speed);

        while(rightfront.isBusy() && rightback.isBusy() && leftfront.isBusy() && leftback.isBusy()) {

        }

        rightfront.setPower(0);
        rightback.setPower(0);
        leftfront.setPower(0);
        leftback.setPower(0);

    }

    public static void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        //leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftfront.setPower(LFPower);
        leftback.setPower(LBPower);
        rightfront.setPower(RFPower);
        rightback.setPower(RBPower);
    }

    public static void initIMU(OpMode opMode){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        opMode.telemetry.addData("Mode: ", "imu calibrating");
        opMode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
        }
        opMode.telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
        opMode.telemetry.update();
        resetAngle();
    }

    public static double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public static void moveToAngle(double angle, double power){
        double  leftPower, rightPower;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (angle > getAngle()) {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (angle < getAngle()) {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        SetPower(leftPower, rightPower, leftPower, rightPower);



        // rotate until turn is completed.
        if(getAngle() > angle){
            while (getAngle() > angle) {}
        } else{
            while (getAngle() < angle) {}
        }

       /* if (angle > getAngle()) {
            // On right turn we have to get off zero first.

            while (getAngle() > angle) {}
        }
        else    // left turn.
            while (getAngle() < angle) {}
        */
        SetPower(0,0,0,0);
    }
    private static void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    public static void launchRingPub(double speed) {
        launcher1.setPower(speed);
        launcher2.setPower(-speed);
        launcherbelt.setPower(0.5);
        sleep(2000);
        launcher1.setPower(0);
        launcher2.setPower(0);
        launcherbelt.setPower(0);

    }
}
