package com.parshwa.drive.tele;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive implements DriveInterface {
    private  Telemetry telemetry;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DriveBuilder builder = new DriveBuilder();
    /**
     * This function is used to move the robot using inputs from controller
     * the forward input controls forward movement
     * the strafe input controls strafing
     * the rotate input controls rotating
     * the speed controls how fast the robot moves
     * **/
    @Override
    public void move(double froward, double strafe, double rotate, double speed) {
        switch (builder.getDriverMode()){
            case Kiwi:
                telemetry.addLine("not Implemented Yet");
                break;
            case HDrive:
                telemetry.addLine("not Implemented Yet");
                break;
            case Swerve:
                telemetry.addLine("not Implemented Yet");
                break;
            case SixWheel:
                telemetry.addLine("not Implemented Yet");
                break;
            case TwoWheel:
                telemetry.addLine("not Implemented Yet");
                break;
            case Holonomic:
                telemetry.addLine("not Implemented Yet");
                break;
            case FourWheel:
                telemetry.addLine("not Implemented Yet");
                break;
            case OneEachSideTank:
                telemetry.addLine("not Implemented Yet");
                break;
            case TwoEachSideTank:
                telemetry.addLine("not Implemented Yet");
                break;
            case MecanumFeildOriented:
                FeildOreintedMode(froward, strafe, rotate, speed);
                telemetry.addLine("Mechanum FeildOreintedDrive is running");
                break;
            case MecanumRobotOriented:
                RobotOrientedMode(froward, strafe, rotate, speed);
                telemetry.addLine("Mechanum RobotOreintedDrive is running");
                break;
            default:
                telemetry.addLine("ERROR INVALID DRIVE TYPE");
                telemetry.addLine("Send me a discord message at parshwa_fun if this is a drive mode type");
                break;
        }
        telemetry.update();
    }
    private void FeildOreintedMode(double forward, double strafe, double rotate, double speed){
        IMU imu = builder.getImu();
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        robotAngle = AngleUnit.normalizeRadians(robotAngle);
        // convert to polar
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(forward, strafe);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);
        RobotOrientedMode(newForward,newStrafe,rotate,speed);
    }
    private void RobotOrientedMode(double forward, double strafe, double rotate, double speed){
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

        leftFrontPower = (forward + strafe + rotate) * speed;
        rightFrontPower= (forward - strafe - rotate) * speed;
        leftBackPower  = (forward - strafe + rotate) * speed;
        rightBackPower = (forward + strafe - rotate) * speed;

        double max = Math.max(1.0,Math.max(rightBackPower,Math.max(rightFrontPower,Math.max(leftBackPower,leftFrontPower))));

        leftFrontPower /= max;
        rightFrontPower/= max;
        leftBackPower  /= max;
        rightBackPower /= max;

        frontRightMotor.setPower(rightFrontPower);
        backRightMotor.setPower(rightBackPower);
        frontLeftMotor.setPower(leftFrontPower);
        backLeftMotor.setPower(leftBackPower);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void FeildOreintedMode(double forward, double strafe, double rotate){
        double speed = builder.getSpeed();
        FeildOreintedMode(forward,strafe,rotate,speed);
    }
    private void RobotOrientedMode(double forward, double strafe, double rotate){
        double speed = builder.getSpeed();
        RobotOrientedMode(forward,strafe,rotate,speed);
    }

    @Override
    /**
     * This function is used to move the robot using inputs from controller
     * the forward input controls forward movement
     * the strafe input controls strafing
     * the rotate input controls rotating
     * the speed controls how fast the robot moves
     * **/
    public void move(double froward, double strafe, double rotate) {
        switch (builder.getDriverMode()){
            case Kiwi:
                telemetry.addLine("not Implemented Yet");
                break;
            case HDrive:
                telemetry.addLine("not Implemented Yet");
                break;
            case Swerve:
                telemetry.addLine("not Implemented Yet");
                break;
            case SixWheel:
                telemetry.addLine("not Implemented Yet");
                break;
            case TwoWheel:
                telemetry.addLine("not Implemented Yet");
                break;
            case Holonomic:
                telemetry.addLine("not Implemented Yet");
                break;
            case FourWheel:
                telemetry.addLine("not Implemented Yet");
                break;
            case OneEachSideTank:
                telemetry.addLine("not Implemented Yet");
                break;
            case TwoEachSideTank:
                telemetry.addLine("not Implemented Yet");
                break;
            case MecanumFeildOriented:
                FeildOreintedMode(froward, strafe, rotate);
                telemetry.addLine("Mechanum FeildOreintedDrive is initilizing");
                break;
            case MecanumRobotOriented:
                RobotOrientedMode(froward, strafe, rotate);
                telemetry.addLine("Mechanum RobotOreintedDrive is initilizing");
                break;
            default:
                telemetry.addLine("ERROR INVALID DRIVE TYPE");
                telemetry.addLine("Send me a discord message at parshwa_fun if this is a drive mode type");
                break;
        }
        telemetry.update();
    }

    @Override
    /**
     * This initilizes all the required parts of this module
     * **/
    public void init(HardwareMap hardwareMap, Telemetry telemetry, DriveModes drivermode) {
        change(drivermode);
        init(hardwareMap,telemetry);
    }

    @Override
    /**
     * This initilizes all the required parts of this module
     * **/
    public void init(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeftMotor = hardwareMap.dcMotor.get(builder.getLeftFront());
        frontRightMotor = hardwareMap.dcMotor.get(builder.getRightFront());
        backLeftMotor = hardwareMap.dcMotor.get(builder.getLeftBack());
        backRightMotor = hardwareMap.dcMotor.get(builder.getRightBack());

        frontLeftMotor.setDirection(builder.getMotorOrientationLeftFront());
        frontRightMotor.setDirection(builder.getMotorOrientationRightFront());
        backLeftMotor.setDirection(builder.getMotorOrientationLeftBack());
        backRightMotor.setDirection(builder.getMotorOrientationRightBack());
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
        telemetry.addLine("Done initilizing drive");
        telemetry.update();
    }
    /**
     * This Changes value so when iniilized you can have right values you need
     * **/
    public void change(double speed){
        builder.speed(speed);
    }
    /**
     * This Changes value so when iniilized you can have right values you need
     * **/
    public void change(IMU imu){
        builder.imu(imu);
    }
    /**
     * This Changes value so when iniilized you can have right values you need
     * **/
    public void change(String rightFront, String rightBack, String leftFront, String leftBack){
        builder.motors(rightFront,rightBack,leftFront,leftBack);
    }
    /**
     * This Changes value so when iniilized you can have right values you need
     * **/
    public void change(DriveModes mode){
        builder.mode(mode);
    }
    /**
     * This Changes value so when iniilized you can have right values you need
     * **/
    public void change(DcMotorSimple.Direction MotorOrientationRightFront, DcMotorSimple.Direction MotorOrientationRightBack, DcMotorSimple.Direction MotorOrientationLeftFront, DcMotorSimple.Direction MotorOrientationLeftBack){
        builder.oreintaion(MotorOrientationRightFront,MotorOrientationRightBack,MotorOrientationLeftFront,MotorOrientationLeftBack);
    }
    public DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }

    public DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }

    public DcMotor getBackLeftMotor() {
        return backLeftMotor;
    }

    public DcMotor getBackRightMotor() {
        return backRightMotor;
    }

}
