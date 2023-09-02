package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class KaitlynRobot{

    public static final double TICKS_PER_INCH = (537.7 / 1.4) / 11.87373601322835;

    public static final double P_CONSTANT = 0.006;


    public static final double P_CONSTANT_TURNING = 0.011;
    private final LinearOpMode opMode;

    HardwareMap robotHardwareMap;
    Telemetry robotTelemetry;

    DcMotor fLeft;
    DcMotor fRight;
    DcMotor bLeft;
    DcMotor bRight;

    public KaitlynRobot(HardwareMap hardwareMap, Telemetry telemetry, KaitlynAuto kaitlynAuto) {
        this.robotHardwareMap = hardwareMap;
        this.robotTelemetry = telemetry;
        this.opMode = kaitlynAuto;

    }

    public void encoderMove (int inches) {
        fLeft = robotHardwareMap.dcMotor.get("fLeft");
        fRight = robotHardwareMap.dcMotor.get("fRight");
        bLeft = robotHardwareMap.dcMotor.get("bLeft");
        bRight = robotHardwareMap.dcMotor.get("bRight");

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

        int ticks = fLeft.getCurrentPosition();
        double error;

        while (ticks <= (TICKS_PER_INCH * Math.abs(inches)) && opMode.opModeIsActive()) {

            ticks = fLeft.getCurrentPosition();
            error = (TICKS_PER_INCH * inches) - ticks;

            robotTelemetry.addLine("moving");
            robotTelemetry.addLine(String.valueOf(ticks));
            robotTelemetry.addLine(String.valueOf(TICKS_PER_INCH * inches));
            robotTelemetry.update();

            fLeft.setPower(P_CONSTANT*error);
            fRight.setPower(P_CONSTANT*error);
            bLeft.setPower(P_CONSTANT*error);
            bRight.setPower(P_CONSTANT*error);

        }

        robotTelemetry.addLine("done");
        robotTelemetry.update();
    }

    public void encoderTurn(int degrees) {

        fLeft = robotHardwareMap.dcMotor.get("fLeft");
        fRight = robotHardwareMap.dcMotor.get("fRight");
        bLeft = robotHardwareMap.dcMotor.get("bLeft");
        bRight = robotHardwareMap.dcMotor.get("bRight");

        //double turnCircumference = 21.8406873747*3.1415926535;
        double turnCircumference = (15) * 3.1415926535;
        double turnNinety = (turnCircumference/2);

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

        double inchesPerTurn = (degrees/90) * turnNinety; //inches

        int ticks = fLeft.getCurrentPosition(); //current ticks
        double error;

        while (ticks <= (TICKS_PER_INCH * Math.abs(inchesPerTurn)) && opMode.opModeIsActive()) {

            ticks = bLeft.getCurrentPosition();
            error = (TICKS_PER_INCH * inchesPerTurn) - ticks;

            //debug
            robotTelemetry.addLine("moving");
            robotTelemetry.addLine(String.valueOf(ticks));
            robotTelemetry.addLine(String.valueOf(TICKS_PER_INCH * inchesPerTurn));
            robotTelemetry.update();

            fLeft.setPower(P_CONSTANT*error);
            fRight.setPower(-P_CONSTANT*error);
            bLeft.setPower(P_CONSTANT*error);
            bRight.setPower(-P_CONSTANT*error);

        }
    }

    public void imuTurn(int degrees){
        IMU imu = robotHardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));


        imu.initialize(parameters);

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

        double yaw   = robotOrientation.getYaw(AngleUnit.DEGREES); //Yaw = rotation around Z-axis (points straight up through logo)
        double pitch = robotOrientation.getPitch(AngleUnit.DEGREES); // Pitch = rotation around X-axis (points toward right side I2C ports)
        double roll  = robotOrientation.getRoll(AngleUnit.DEGREES); // Roll = rotation around Y-axis (points towards top edge USB ports)

        double degreesToTurn = degrees;

        fLeft = robotHardwareMap.dcMotor.get("fLeft");
        fRight = robotHardwareMap.dcMotor.get("fRight");
        bLeft = robotHardwareMap.dcMotor.get("bLeft");
        bRight = robotHardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double error;

        imu.resetYaw();

        robotTelemetry.addLine("reset done");
        robotTelemetry.update();


        while ((Math.abs(yaw) < Math.abs(degreesToTurn)) && opMode.opModeIsActive()) {
            error = degreesToTurn + yaw;

            fLeft.setPower(P_CONSTANT_TURNING*error);
            fRight.setPower(-P_CONSTANT_TURNING*error);
            bLeft.setPower(P_CONSTANT_TURNING*error);
            bRight.setPower(-P_CONSTANT_TURNING*error);

            robotOrientation = imu.getRobotYawPitchRollAngles();

            yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);

            robotTelemetry.addLine(String.valueOf(robotOrientation.getYaw(AngleUnit.DEGREES)));
            robotTelemetry.addLine(String.valueOf(robotOrientation.getPitch(AngleUnit.DEGREES)));
            robotTelemetry.addLine(String.valueOf(robotOrientation.getRoll(AngleUnit.DEGREES)));
            robotTelemetry.addLine(String.valueOf(degreesToTurn));
            robotTelemetry.addLine(String.valueOf(error));
            robotTelemetry.update();

        }

    }

    public void mecanumStrafe(int inches) {
        fLeft = robotHardwareMap.dcMotor.get("fLeft");
        fRight = robotHardwareMap.dcMotor.get("fRight");
        bLeft = robotHardwareMap.dcMotor.get("bLeft");
        bRight = robotHardwareMap.dcMotor.get("bRight");

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

        int ticks = fLeft.getCurrentPosition();
        double error;

        while (ticks <= (TICKS_PER_INCH * Math.abs(inches)) && opMode.opModeIsActive()) {

            ticks = fLeft.getCurrentPosition();
            error = (TICKS_PER_INCH * inches) - ticks;

            robotTelemetry.addLine("moving");
            robotTelemetry.addLine(String.valueOf(ticks));
            robotTelemetry.addLine(String.valueOf(TICKS_PER_INCH * inches));
            robotTelemetry.update();

            fLeft.setPower(P_CONSTANT*error);
            fRight.setPower(-P_CONSTANT*error);
            bLeft.setPower(-P_CONSTANT*error);
            bRight.setPower(P_CONSTANT*error);

        }

        robotTelemetry.addLine("done");
        robotTelemetry.update();
    }

}
