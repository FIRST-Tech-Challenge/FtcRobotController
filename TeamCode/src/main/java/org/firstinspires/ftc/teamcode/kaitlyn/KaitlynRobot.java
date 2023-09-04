package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class KaitlynRobot{

    public static final double TICKS_PER_INCH = (537.7 / 1.4) / 11.87373601322835;

    public static final double P_CONSTANT_TURNING = 0.01;
    public static final double P_CONSTANT = 0.0012;
    public static final double I_CONSTANT = 0.00000000001;
    public static final double D_CONSTANT = 0.000000000000000001;
    private final LinearOpMode opMode;

    HardwareMap robotHardwareMap;
    Telemetry robotTelemetry;

    DcMotor fLeft;
    DcMotor fRight;
    DcMotor bLeft;
    DcMotor bRight;

    ElapsedTime elapsedTime = new ElapsedTime();

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

        elapsedTime.reset();
        double oldTick = 0;

        while (opMode.opModeIsActive()) {

            ticks = fLeft.getCurrentPosition();
            error = (TICKS_PER_INCH * inches) - ticks;

            robotTelemetry.addLine(String.valueOf(error));
            robotTelemetry.addLine("moving");
            robotTelemetry.addLine(String.valueOf(ticks));
            robotTelemetry.addLine(String.valueOf(TICKS_PER_INCH * inches));
            robotTelemetry.update();

            setAllPower(P_CONSTANT*error);

            if (elapsedTime.milliseconds() >= 500) {

                double newTick = fLeft.getCurrentPosition();


                if (newTick == oldTick) {
                    break;
                }

                oldTick = newTick;

                elapsedTime.reset();

            }

        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

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
        double oldTick = 0;
        elapsedTime.reset();

        while (opMode.opModeIsActive()) {

            ticks = bLeft.getCurrentPosition();
            error = (TICKS_PER_INCH * inchesPerTurn) - ticks;

            //debug
            robotTelemetry.addLine("moving");
            robotTelemetry.addLine(String.valueOf(ticks));
            robotTelemetry.addLine(String.valueOf(TICKS_PER_INCH * inchesPerTurn));
            robotTelemetry.update();

            setTurnPowers(P_CONSTANT_TURNING*error, -P_CONSTANT_TURNING*error);

            if (elapsedTime.milliseconds() >= 500) {

                double newTick = fLeft.getCurrentPosition();


                if (newTick == oldTick) {
                    break;
                }

                oldTick = newTick;

                elapsedTime.reset();

            }

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

            setTurnPowers(P_CONSTANT_TURNING*error, -P_CONSTANT_TURNING*error);

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
        double oldTick = 0;
        elapsedTime.reset();

        while (opMode.opModeIsActive()) {

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

            if (elapsedTime.milliseconds() >= 500) {

                double newTick = fLeft.getCurrentPosition();


                if (newTick == oldTick) {
                    break;
                }

                oldTick = newTick;

                elapsedTime.reset();

            }

        }

        robotTelemetry.addLine("done");
        robotTelemetry.update();
    }

    public void imuTurnWithPID(int degrees) {
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
        double errorSum = 0;
        double prevError = degreesToTurn + yaw;
        double errorDer;
        double newTimeStamp;

        imu.resetYaw();

        robotTelemetry.addLine("reset done");
        robotTelemetry.update();

        double lastTimeStamp = 0;
        ElapsedTime timer = new ElapsedTime();


        while ((Math.abs(yaw) < Math.abs(degreesToTurn)) && opMode.opModeIsActive()) {
            newTimeStamp = timer.milliseconds();
            error = degreesToTurn + yaw;
            errorSum += error * (newTimeStamp - lastTimeStamp);
            errorDer = (error - prevError) / (newTimeStamp - lastTimeStamp);

            setTurnPowers(P_CONSTANT_TURNING*error + I_CONSTANT*errorSum + D_CONSTANT*errorDer, -P_CONSTANT*error - I_CONSTANT*errorSum - D_CONSTANT*errorDer);

            robotOrientation = imu.getRobotYawPitchRollAngles();

            yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);

            robotTelemetry.addLine(String.valueOf(robotOrientation.getYaw(AngleUnit.DEGREES)));
            robotTelemetry.addLine(String.valueOf(robotOrientation.getPitch(AngleUnit.DEGREES)));
            robotTelemetry.addLine(String.valueOf(robotOrientation.getRoll(AngleUnit.DEGREES)));
            robotTelemetry.addLine(String.valueOf(degreesToTurn));
            robotTelemetry.addLine(String.valueOf(error));
            robotTelemetry.update();

            prevError = error;

            lastTimeStamp = newTimeStamp;

        }
    }

    public void encoderMoveWithPID(int inches) {
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
        double error = (TICKS_PER_INCH * inches) - ticks;
        double errorSum = 0;
        double prevError = TICKS_PER_INCH*inches - ticks;
        double errorDer;
        double newTimeStamp;

        double lastTimeStamp = 0;
        ElapsedTime timer = new ElapsedTime();

        elapsedTime.reset();
        double oldTick = 0;

        while (opMode.opModeIsActive()) {
            newTimeStamp = timer.milliseconds();
            ticks = fLeft.getCurrentPosition();
            error = (TICKS_PER_INCH * inches) - ticks;
            errorSum += error * (newTimeStamp - lastTimeStamp);
            errorDer = (error - prevError) / (newTimeStamp - lastTimeStamp);

            robotTelemetry.addLine("moving");
            robotTelemetry.addLine(String.valueOf(ticks));
            robotTelemetry.addLine(String.valueOf(TICKS_PER_INCH * inches));
            robotTelemetry.update();

            setAllPower(P_CONSTANT*error + I_CONSTANT*errorSum + D_CONSTANT*errorDer);

            if (elapsedTime.milliseconds() >= 500) {

                double newTick = fLeft.getCurrentPosition();


                if (newTick == oldTick) {
                    break;
                }

                oldTick = newTick;

                elapsedTime.reset();

            }

        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

        robotTelemetry.addLine("done");
        robotTelemetry.update();
    }

    public void setStraightPower(double fLeftPower, double fRightPower, double bLeftPower, double bRightPower) {
        fLeft.setPower(fLeftPower);
        fRight.setPower(fRightPower);
        bLeft.setPower(bLeftPower);
        bRight.setPower(bRightPower);
    }

    public void setAllPower(double allPowers) {
        setStraightPower(allPowers, allPowers, allPowers, allPowers);
    }

    public void setTurnPowers(double leftPowers, double rightPowers) {
        setStraightPower(leftPowers, rightPowers, leftPowers, rightPowers);
    }

}
