package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//import servo
import com.qualcomm.robotcore.hardware.Servo;

//used for reading IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

abstract public class BaseOpMode extends LinearOpMode {

    //Declared drive motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    //declare turntable and slide motors
    DcMotor motorTurnTable;


    //declare servo
    Servo servoGrabber;

    //declare imu sensor
    BNO055IMU imu;
    //original angle reading from imu that will
    //be used to find unwanted angle offset during drive
    Orientation IMUOriginalAngles;
    //constant for converting angle error to motor speed
    double correctionConstant = 1/45.0;

    public void initHardware() {
        //init drive motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        //reset encoder
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //invert some motors to make programming them more straightforward
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        //init turntable and slide motors
        //motorTurnTable = hardwareMap.dcMotor.get("motorTurnTable");

        //init servo
        servoGrabber = hardwareMap.servo.get("servoGrabber");

        //create parameters for IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        //init IMU sensor
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //preset the IMUAngles so it doesn't start on null
        //since it will only later be read when turning
        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    boolean turnFlag = false; // Flag to say whether we should disable the correction system

    public void driveRobot(double x, double y, double t)  {

        telemetry.addData("org: ", IMUOriginalAngles.firstAngle);//temp
        telemetry.addData("t1=", t);//temp
        //read imu when turning (when t != 0)

        boolean isTurning = t != 0; // Should we disable the correction based on whether the robot is turning because of user input
        if (isTurning || turnFlag) {
            IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Set original angle
            if(!turnFlag) { // If the robot is turning because of user input and the disable flag is false
                turnFlag = true; // Set to true
            }
        //otherwise read imu for correction
        } else {
            //obtain the current angle's error from the original angle
            Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angleError = IMUOriginalAngles.firstAngle - currentAngle.firstAngle;
            //flip to inverse of angles above 180/below -180 (to prevent funny infinirotate bug)
            //to make sure to use the shorter angle
            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }
            telemetry.addData("error=", angleError);//temp
            //apply a constant to turn the angle into a turn speed

            t = -correctionConstant * angleError;
        }

        if(Math.abs(imu.getAngularVelocity().zRotationRate) < 5) { // If the rotation rate is low,
            // then that means all the momentum has left the robot's turning and can therefore turn the correction back on
            turnFlag = false;
        }

        telemetry.addData("t2=", t);//temp
        telemetry.addLine("hello, this is cool!");//temp
        telemetry.update();//temp

        //calculate speed and direction of each individual motor
        double speedFL = (-y+x+t);
        double speedFR = (-y-x-t);
        double speedBL = (-y-x+t);
        double speedBR = (-y+x-t);

        //set power of motors to speed
        motorFL.setPower(speedFL);
        motorFR.setPower(speedFR);
        motorBL.setPower(speedBL);
        motorBR.setPower(speedBR);

        telemetry.addData("IMU disabled flag", turnFlag);
        telemetry.addData("z rot vel", imu.getAngularVelocity().zRotationRate);

    }

    //method to open/close grabber if you set boolean to true or false
    //this is so that you can use it in Autonomous and TeleOp.
    public void openGrabber(boolean open) {
        if (open) {
            servoGrabber.setPosition(0.33); //set servo to open position
        } else {
            servoGrabber.setPosition(0.11); //set servo to closed position
        }
    }
    //this is a general method to turn the turntable
    public void rotateTurntable(double power){
        motorTurnTable.setPower(power);
    }
}
