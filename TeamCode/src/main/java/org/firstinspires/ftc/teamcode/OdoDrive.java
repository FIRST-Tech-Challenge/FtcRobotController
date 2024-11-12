package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;

@Config //Dash
@TeleOp(name="OdoDrive")
public class OdoDrive extends OpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotorEx test;
    private IMU gyro;

    double leftOffset, rightOffset, auxOffset;

    public DcMotor leftEncoder, rightEncoder, auxEncoder;

    //private Odometry Odometry;

    private double xCoord = 0, yCoord = 0, heading = 0;

    private double lastLeftPos = 0, lastRightPos = 0, lastAuxPos = 0;

    double TICKS_PER_INCH = 8192 / ((35 / 25.4) * Math.PI); //  1892.37   1946.69
    double encoderDistance = 8.5;
    double auxEncoderOffset = -2.48;

    double leftPos, rightPos, auxPos, deltaLeft, deltaHeading, deltaX, deltaY;

    @Override
    public void init(){

        //Drive Motor Setup - Used the name from FeverDream
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        backLeft = hardwareMap.dcMotor.get("BackLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");
        backRight = hardwareMap.dcMotor.get("BackRight");

        //frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        //Dead Wheel Setup
        leftEncoder = hardwareMap.get(DcMotor.class, "BackLeft"); //Exp: 2
        rightEncoder = hardwareMap.get(DcMotor.class, "FrontRight"); //Exp: 3
        auxEncoder = hardwareMap.get(DcMotor.class, "BackRight"); //Hub: 2

        //Setup and start Odometry Thread
        /*Odometry = new Odometry(
                leftEncoder,
                rightEncoder,
                auxEncoder,
                946.1843217,//234.05714,
                12,
                5);
        Thread odometryThread = new Thread(Odometry);
        odometryThread.start();*/

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);

    }

    @Override
    public void loop() {
        leftPos = leftEncoder.getCurrentPosition() / TICKS_PER_INCH;
        rightPos = rightEncoder.getCurrentPosition() / TICKS_PER_INCH;
        auxPos = auxEncoder.getCurrentPosition() / TICKS_PER_INCH;

        if (gamepad1.a) {
            resetEncoders();
        }

        // Offsets the encoders position to zero when zero function is updated.
        if(leftOffset >= 0){
            leftPos -= leftOffset;
        } else {
            leftPos += leftOffset;
        }
        if(rightOffset >= 0){
            rightPos -= rightOffset;
        } else {
            rightPos += rightOffset;
        }
        if(auxOffset >= 0){
            auxPos -= auxOffset;
        } else {
            auxPos += auxOffset;
        }

        double deltaLeft = leftPos - lastLeftPos;
        double deltaRight = rightPos - lastRightPos;
        double deltaAux = auxPos - lastAuxPos;

        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        lastAuxPos = auxPos;

        //Find the heading in radians - 5225 Position Docs Equations
        deltaHeading = (deltaLeft - deltaRight) / encoderDistance;
        heading += deltaHeading;

        deltaX = deltaAux - (auxEncoderOffset * deltaHeading); ///changed to a negative, as show in GM0
        deltaY = (deltaLeft + deltaRight) / 2;

        xCoord += deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        yCoord += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);

        double robotHeading = Math.toDegrees(heading);

        //Loop heading from 0-360 deg
        robotHeading = Math.abs(robotHeading % 360);

        double y = -gamepad1.left_stick_y;//branch
        double x = gamepad1.left_stick_x;
        double h = -gamepad1.right_stick_x;

        double finalX = x * Math.cos(heading) - y * Math.sin(heading);
        double finalY = x * Math.sin(heading) + y * Math.cos(heading);

        //Set motor speeds
        double denominator = Math.max(Math.abs(finalY) + Math.abs(finalX) + Math.abs(h), 1);
        double FLPower = (finalY + finalX + h) / denominator;
        double BLPower = (finalY - finalX + h) / denominator;
        double FRPower = (finalY - finalX - h) / denominator;
        double BRPower = (finalY + finalX - h) / denominator;

        frontLeft.setPower(FLPower);
        backLeft.setPower(BLPower);
        frontRight.setPower(FRPower);
        backRight.setPower(BRPower);

        telemetry.addData("X Pos:", xCoord);
        telemetry.addData("Y Pos", yCoord);
        telemetry.addData("Heading", robotHeading); //Long live Bird Nest
        telemetry.addData("Enc Left ", leftEncoder.getCurrentPosition());
        telemetry.addData("Enc Right", rightEncoder.getCurrentPosition());
        telemetry.addData("Enc Aux  ", auxEncoder.getCurrentPosition());

    }

    public void debugData(){
        ///// Debug /////

        telemetry.addData("leftPos", leftPos);
        telemetry.addData("deltaLeft", deltaLeft);
        telemetry.addData("lastleftPos", lastLeftPos);
        telemetry.addData("dHeading", deltaHeading);
        telemetry.addData("heading", heading);
        telemetry.addData("deltaX", deltaX);
        telemetry.addData("deltaY", deltaY);
    }

    public void resetEncoders(){
        //if in imu mode
        gyro.resetYaw();

        //If in DeadWheel mode
        leftOffset = leftEncoder.getCurrentPosition();
        rightOffset = rightEncoder.getCurrentPosition();
        auxOffset = auxEncoder.getCurrentPosition();

        lastLeftPos = 0;
        lastRightPos = 0;
        lastAuxPos = 0;
    }
}
