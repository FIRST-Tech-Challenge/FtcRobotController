package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

public class TeleOp extends OpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private IMU gyro;

    public DcMotor leftEncoder, rightEncoder, auxEncoder;

    private Odometry Odometry;

    private double xCoord = 0, yCoord = 0, heading = 0;

    private double lastLeftPos = 0, lastRightPos = 0, lastAuxPos = 0;

    double COUNTS_PER_INCH = 5945.0514285952;
    double encoderDistance = 8.5;
    double auxEncoderOffset = -2.48;

    @Override
    public void init(){
        //Drive Motor Setup - Used the name from FeverDream
        frontLeft = hardwareMap.dcMotor.get("FrontLeftDrive");
        backLeft = hardwareMap.dcMotor.get("BackLeftDrive");
        frontRight = hardwareMap.dcMotor.get("FrontRightDrive");
        backRight = hardwareMap.dcMotor.get("BackRightDrive");

        //frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        //Dead Wheel Setup
        leftEncoder = hardwareMap.get(DcMotor.class, "IntakePivot"); //Exp: 2
        rightEncoder = hardwareMap.get(DcMotor.class, "IntakeMotor"); //Exp: 3
        auxEncoder = hardwareMap.get(DcMotor.class, "LeftSlide"); //Hub: 2

        //Setup and start Odometry Thread
        Odometry = new Odometry(
                leftEncoder,
                rightEncoder,
                auxEncoder,
                946.1843217,//234.05714,
                12,
                5);
        Thread odometryThread = new Thread(Odometry);
        odometryThread.start();

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);

    }

    @Override
    public void loop() {
        double leftPos = leftEncoder.getCurrentPosition();
        double rightPos = rightEncoder.getCurrentPosition();
        double auxPos = auxEncoder.getCurrentPosition();


        //telemetry.addData("leftPos", leftPos);

        double deltaLeft = leftPos - lastLeftPos;
        double deltaRight = rightPos - lastRightPos;
        double deltaAux = auxPos - lastAuxPos;

        //telemetry.addData("deltaLeft", deltaLeft);

        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        lastAuxPos = auxPos;

        //telemetry.addData("lastleftPOs", lastLeftPos);

        //Find the heading in radians - 5225 Position Docs Equations
        double deltaHeading = (deltaLeft - deltaRight) / encoderDistance;
        heading += deltaHeading;

        //telemetry.addData("dHeading", deltaHeading);
        //telemetry.addData("heading", heading);

        double deltaX = deltaAux + (auxEncoderOffset * deltaHeading);
        double deltaY = (deltaLeft + deltaRight) / 2;

        //telemetry.addData("deltaX", deltaX);
        //telemetry.addData("deltaY", deltaY);

        xCoord += deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        yCoord += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);

        double robotHeading = Math.toRadians(heading); //Might need degrees???
        //double robotHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double h = -gamepad1.right_stick_x;

        if (gamepad1.a) {
            gyro.resetYaw();
            //not sure if all of these need to be reset
            xCoord = 0;
            yCoord = 0;
            heading = 0;
            auxPos = 0;
            rightPos = 0;
            leftPos = 0;
            lastLeftPos = 0;
            lastRightPos = 0;
            lastAuxPos = 0;
            deltaLeft = 0;
            deltaRight = 0;
            deltaAux = 0;
            deltaX = 0;
            deltaY = 0;
            deltaHeading = 0;
            robotHeading = 0;
        }

        double finalX = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        double finalY = x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

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
        telemetry.addData("Heading", Math.toDegrees(robotHeading));
        telemetry.addData("Enc Left ", leftEncoder.getCurrentPosition());
        telemetry.addData("Enc Right", rightEncoder.getCurrentPosition());
        telemetry.addData("Enc Aux  ", auxEncoder.getCurrentPosition());
    }
}
