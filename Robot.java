package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Robot {

    //initialization
    double x=0;
    double y=0;
    double angle=0;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;


    private DcMotor arm;
    Odometry odometry = new Odometry();
    public void init(HardwareMap hardwareMap,boolean wheels, boolean od, boolean a){
        // Declare motors
        if (wheels){
            frontLeftMotor = hardwareMap.dcMotor.get("FL");
            backLeftMotor = hardwareMap.dcMotor.get("BL");
            frontRightMotor = hardwareMap.dcMotor.get("FR");
            backRightMotor = hardwareMap.dcMotor.get("BR");

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        if (od){
            leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
            rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
            frontEncoder = hardwareMap.dcMotor.get("frontEncoder");

            rightEncoder.setDirection(DcMotor.Direction.REVERSE);
            frontEncoder.setDirection(DcMotor.Direction.REVERSE);
        }


        if (a){
            arm = hardwareMap.dcMotor.get("A");
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }


    }

    public void updatePosition(){
        int leftEncoderPos = leftEncoder.getCurrentPosition();
        int rightEncoderPos = rightEncoder.getCurrentPosition();
        int frontEncoderPos = frontEncoder.getCurrentPosition();

        odometry.updateOdometry(leftEncoderPos, rightEncoderPos, frontEncoderPos);
        x=odometry.getX();
        y=odometry.getY();
        angle=odometry.getAngle();
    }

    public void move(double x, double y, double rx){
        double frontLeftPower = Math.min(y + x + rx, 1);
        double backLeftPower = Math.min(y - x + rx, 1);
        double frontRightPower = Math.min(y - x - rx, 1);
        double backRightPower = Math.min(y + x - rx, 1);

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void moveArm(int x, LinearOpMode opMode){

        arm.setTargetPosition(x);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        while (opMode.opModeIsActive() && arm.isBusy()){
            opMode.telemetry.addData("Arm Position:", arm.getCurrentPosition());
        }

        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getAngle() {
        return angle;
    }

    public double getArmPosition(){return arm.getCurrentPosition();}
}
