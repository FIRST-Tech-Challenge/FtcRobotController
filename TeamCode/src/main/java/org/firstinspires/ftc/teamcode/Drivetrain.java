package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static java.lang.Math.*;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Drivetrain{

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private BNO055IMU imu;
    private Telemetry telemetry;
    static final int TickPerRev = 10; // need to measure
    double angleOffset = 0;

    Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        this.telemetry = telemetry;

    }

    public enum DirectionMode{
        FORWARD, SIDE, ROTATE
    }

    public enum MotorQuality{
        POW, POS, VOL
    }

    public enum MotorPlacement{
        LEFTFRONT, LEFTBACK, RIGHTFRONT, RIGHTBACK
    }

    public enum Centricity{
        BOT, FIELD
    }

    private int[] DetermineDirection(DirectionMode mode){
        int[] directions = new int[]{1, 1, 1, 1};

        switch (mode) {
            case FORWARD:
                break;

            case SIDE:
                directions[1] = -1;
                directions[2] = -1;
                break;

            case ROTATE:
                directions[2] = -1;
                directions[3] = -1;
                break;
        }

        return directions;
    }

    public void setMotorPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower){
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void setMotorPowers(double[] motorPowers){
        leftFrontDrive.setPower(motorPowers[0]);
        leftBackDrive.setPower(motorPowers[1]);
        rightFrontDrive.setPower(motorPowers[2]);
        rightBackDrive.setPower(motorPowers[3]);
    }

    public void setIndividualPower(MotorPlacement pos, double pow) {
        switch (pos) {
            case LEFTFRONT:
                leftFrontDrive.setPower(pow);
                break;
            case LEFTBACK:
                leftBackDrive.setPower(pow);
                break;
            case RIGHTFRONT:
                leftFrontDrive.setPower(pow);
                break;
            case RIGHTBACK:
                leftBackDrive.setPower(pow);
                break;
        }
    }

    public void move(DirectionMode mode, double pow) {

        int[] directions = DetermineDirection(mode);

        setMotorPowers(directions[0] * pow, directions[1] * pow, directions[2] * pow, directions[3] * pow);
    }

    public double getMotorInfo(MotorPlacement pos, MotorQuality qual){
        DcMotor workingMotor;
        switch (pos){
            case LEFTFRONT:
                workingMotor = leftFrontDrive;
                break;
            case LEFTBACK:
                workingMotor = leftBackDrive;
                break;
            case RIGHTFRONT:
                workingMotor = rightFrontDrive;
                break;
            case RIGHTBACK:
                workingMotor = rightBackDrive;
                break;
            default:
                workingMotor = leftFrontDrive;
        }

        switch (qual){
            case POS:
                return workingMotor.getCurrentPosition();
            case VOL:
            case POW:
            default:
                return workingMotor.getPower();
        }

    }

    //rather confusingly this just gives the heading
    public double getIMUData(){
        return -imu.getAngularOrientation().firstAngle+angleOffset;
    }

    public void resetAngle(){
        angleOffset = imu.getAngularOrientation().firstAngle;
    }

    public void setAngleOffset(double angle){
        angleOffset = angle*PI/180;
    }


    //put in the three vectors (z being rotation) and it will move the motors the correct powers
    public double[] driveVectors(Centricity centric, double joyx, double joyy, double joyz, double spd){ //spd is a speed coefficient

        //set up movement vectors and relate them to input ("joy") vectors
        double x, y, max;
        if(centric==Centricity.FIELD){
            double angle = getIMUData();

            x = joyx*cos(angle) - joyy*sin(angle);
            y = joyx*sin(angle) + joyy*cos(angle);
        }
        else{
            x = joyx;
            y = joyy;
        }

        //set up the list of motor powers, related to the movement vectors, z being rotation
        double[] motorPowers = new double[]{
                (x + y + joyz)*spd,
                (-x + y + joyz)*spd,
                (-x + y - joyz)*spd,
                (x + y - joyz)*spd
        };

        //make sure no individual motor powers are higher than 1
        max = 0;
        for (double candidate:motorPowers){
            if(abs(candidate)>max){max = abs(candidate);}
        }

        if(max>1){
            for(int i=0; i<4; i++){
                motorPowers[i] /= max;
            }
        }

        //set the motor powers
        setMotorPowers(motorPowers);

        return motorPowers;
    }

    // this function is designed for the auto part
    public void MoveForDis(double distance, double pow) throws InterruptedException {

        // calculate the distance
        int dis = (int)(distance * 1000 / 23.5 * 50/48);

        // reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set target distance
        leftFrontDrive.setTargetPosition(dis);
        leftBackDrive.setTargetPosition(dis);
        rightFrontDrive.setTargetPosition(dis);
        rightBackDrive.setTargetPosition(dis);

        // change encoder mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set power
        for(int i =1; i<21; i++) {
            leftFrontDrive.setPower(pow*i/20);
            leftBackDrive.setPower(pow*i/20);
            rightFrontDrive.setPower(pow*i/20);
            rightBackDrive.setPower(pow*i/20);
            sleep(10);
        }

        // run for a while
        while ( leftFrontDrive.isBusy() ||
                leftBackDrive.isBusy() ||
                rightFrontDrive.isBusy() ||
                rightBackDrive.isBusy() ) {}


        // change encoder mode back to normal
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // set power to 0
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

    // this function is designed for the auto part
    public void SideMoveForDis(double distance, double pow) {

        // calculate the distance
        int dis = (int)(distance * 1000 / 19.75);

        // set direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set target distance
        leftFrontDrive.setTargetPosition(dis);
        leftBackDrive.setTargetPosition(dis);
        rightFrontDrive.setTargetPosition(dis);
        rightBackDrive.setTargetPosition(dis);

        // change encoder mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set power
        leftFrontDrive.setPower(pow);
        leftBackDrive.setPower(pow);
        rightFrontDrive.setPower(pow);
        rightBackDrive.setPower(pow);

        // run for a while
        while ( leftFrontDrive.isBusy() ||
                leftBackDrive.isBusy() ||
                rightFrontDrive.isBusy() ||
                rightBackDrive.isBusy() ) {}

        // set power to 0
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // change encoder mode back to normal
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // set power to 0
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // this function is designed for the auto part
    public void RotateForDegree(int degree, double pow) {

        // calculate the degree
        int deg = (int)(degree * 1000 / 105);

        // reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set direction
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // set target distance
        leftFrontDrive.setTargetPosition(deg);
        leftBackDrive.setTargetPosition(deg);
        rightFrontDrive.setTargetPosition(deg);
        rightBackDrive.setTargetPosition(deg);

        // change encoder mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set power
        leftFrontDrive.setPower(pow);
        leftBackDrive.setPower(pow);
        rightFrontDrive.setPower(-pow);
        rightBackDrive.setPower(-pow);

        // run for a while
        while ( leftFrontDrive.isBusy() ||
                leftBackDrive.isBusy() ||
                rightFrontDrive.isBusy() ||
                rightBackDrive.isBusy() ) {}

        // set power to 0
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // change encoder mode back to normal
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void rotateToPosition(double targetAngle, double pow){
        double angle = getIMUData()*180/PI;
        double angleMod = 0;
        double speedCoef = 1;

        if(abs(targetAngle-angle)<abs(targetAngle-(angle-360))){
            if(abs(targetAngle-angle)<abs(targetAngle-(angle+360))){
                angleMod=0;
            }else{
                angleMod=360;
            }
        }else{
            angleMod=-360;
        }

        while (abs(targetAngle-(angle+angleMod))>0.2){
            angle = getIMUData()*180/PI;

            if (abs(targetAngle-(angle+angleMod))<=30){
                speedCoef = abs(targetAngle-(angle+angleMod))/30;
            }
            else{
                speedCoef=1;
            }

            move(DirectionMode.ROTATE, abs(pow) * ((targetAngle-(angle+angleMod)) / abs(targetAngle-(angle+angleMod))) * speedCoef);


            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Actual Angle", (angle+angleMod));

            telemetry.update();
        }
    }
}