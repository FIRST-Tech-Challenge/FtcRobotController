package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class KaitlynAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        imuTurn(90);
    }

    public static final double TICKS_PER_INCH = (537.7 / 1.4) / 11.87373601322835;
    public static final double P_CONSTANT = 0.002;

    public void encoderMove(int inches) {

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        int ticks = fLeft.getCurrentPosition();
        double error;

        while (ticks <= (TICKS_PER_INCH * Math.abs(inches)) && opModeIsActive()) {

            ticks = fLeft.getCurrentPosition();
            error = (TICKS_PER_INCH * inches) - ticks;

            telemetry.addLine("moving");
            telemetry.addLine(String.valueOf(ticks));
            telemetry.addLine(String.valueOf(TICKS_PER_INCH * inches));
            telemetry.update();

            fLeft.setPower(P_CONSTANT*error);
            fRight.setPower(P_CONSTANT*error);
            bLeft.setPower(P_CONSTANT*error);
            bRight.setPower(P_CONSTANT*error);

        }

        telemetry.addLine("done");
        telemetry.update();

    }

    public void encoderTurn(int degrees) {

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        //double turnCircumference = 21.8406873747*3.1415926535;
        double turnCircumference = (15) * 3.1415926535;
        double turnNinety = (turnCircumference/2);

        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double inchesPerTurn = (degrees/90) * turnNinety; //inches

        int ticks = bLeft.getCurrentPosition(); //current ticks
        double error;

        waitForStart();

        while (ticks <= (TICKS_PER_INCH * Math.abs(inchesPerTurn)) && opModeIsActive()) {

            ticks = bLeft.getCurrentPosition();
            error = (TICKS_PER_INCH * inchesPerTurn) - ticks;

            //debug
            telemetry.addLine("moving");
            telemetry.addLine(String.valueOf(ticks));
            telemetry.addLine(String.valueOf(TICKS_PER_INCH * inchesPerTurn));
            telemetry.update();

            fLeft.setPower(P_CONSTANT*error);
            fRight.setPower(-P_CONSTANT*error);
            bLeft.setPower(P_CONSTANT*error);
            bRight.setPower(-P_CONSTANT*error);

        }
    }

    public void imuTurn(int degrees){
        final BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES); //Yaw = rotation around Z-axis (points straight up through logo)
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES); // Pitch = rotation around X-axis (points toward right side I2C ports)
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES); // Roll = rotation around Y-axis (points towards top edge USB ports)

        double degreesToTurn = -degrees;

        DcMotor fLeft = hardwareMap.dcMotor.get("fLeft");
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft");
        DcMotor bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double error;

        waitForStart();

        while ((Math.abs(Yaw) < Math.abs(degreesToTurn)) && opModeIsActive()) {
            error = degreesToTurn - Yaw;

            fLeft.setPower(P_CONSTANT*error);
            fRight.setPower(-P_CONSTANT*error);
            bLeft.setPower(P_CONSTANT*error);
            bRight.setPower(-P_CONSTANT*error);
        }

    }

    public void wait(int seconds) {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (true) {

            if (elapsedTime.milliseconds() >= seconds*1000) {
                break;
            }

        }
    }
}

