package org.firstinspires.ftc.teamcode.Ethan;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class EthanRobot {

    // CLASS PROPERTIES
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    DcMotor lFront;
    DcMotor rFront;
    DcMotor lBack;
    DcMotor rBack;
    DcMotor arm;
    Servo lServo;
    Servo rServo;
    IMU imu;

    double oldTick;
    long millis = System.currentTimeMillis();
    long lastCheckMillis = System.currentTimeMillis();

    // CONSTRUCTOR
    public EthanRobot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
        setUpMotors();
    }

    public void setUpMotors() {
        lFront = hardwareMap.dcMotor.get("Left front");
        rFront = hardwareMap.dcMotor.get("Right front");
        lBack = hardwareMap.dcMotor.get("Left back");
        rBack = hardwareMap.dcMotor.get("Right back");

        rBack.setDirection(DcMotorSimple.Direction.FORWARD);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        lFront.setDirection(DcMotorSimple.Direction.REVERSE);


        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setUpImu() {

        this.imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();


    }

    public void setUpArmAndServo() {

        arm = hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        this.lServo = hardwareMap.servo.get("lServo");
        this.rServo = hardwareMap.servo.get("rServo");
    }

    public void setMotorPower(double lFront, double rFront, double lBack, double rBack) {
        this.lFront.setPower(lFront);
        this.rFront.setPower(rFront);
        this.lBack.setPower(lBack);
        this.rBack.setPower(rBack);
    }

    public void setArmPower(double armPower) {
        arm.setPower(armPower);
    }

    public void setServoPosition(@NonNull Boolean servoPosition) {

        if (servoPosition) {
            lServo.setPosition(1);
            rServo.setPosition(0);
        }

        if (servoPosition) {
            lServo.setPosition(0);
            rServo.setPosition(1);
        }

    }

    public void autoForward(double targetDistanceInMM) throws InterruptedException {


        final double P_VALUE = 0.006;

        //301 = circumferance mm
        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7 / 1.4) / 301.59;

        double targetPos = targetDistanceInMM * MM_TO_TICKS + lFront.getCurrentPosition();
        double error = targetPos - lFront.getCurrentPosition();

        final double PROPORTIONAL_POWER = P_VALUE * error;



        /*lFront.setPower(0.1);
        lBack.setPower(0.1);
        rFront.setPower(0.1);
        rBack.setPower(0.1);

        while (lFront.getCurrentPosition() < targetPos && opModeIsActive()) {}

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);*/


        long lastCheckMillis = System.currentTimeMillis();
        long millis = System.currentTimeMillis();

        double oldTick = lFront.getCurrentPosition();

        boolean isStopped = false;

        while (!(error <= 10 && isStopped) && opMode.opModeIsActive()) {

            millis = System.currentTimeMillis();

            telemetry.addLine(String.valueOf(targetPos));
            telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));

            error = targetPos - lFront.getCurrentPosition();
            telemetry.addLine(String.valueOf(error));

            lFront.setPower(PROPORTIONAL_POWER);
            lBack.setPower(PROPORTIONAL_POWER);
            rFront.setPower(PROPORTIONAL_POWER);
            rBack.setPower(PROPORTIONAL_POWER);

            telemetry.addLine("moving");


            if (millis > lastCheckMillis + 500) {
                lastCheckMillis = millis;
                double newTicks = lFront.getCurrentPosition();

                if (oldTick == newTicks) {
                    isStopped = true;
                }

                oldTick = newTicks;


            }


            telemetry.update();
        }

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        telemetry.addLine(String.valueOf(lFront.getCurrentPosition() / MM_TO_TICKS));

        /*double oldTick = 0;

        while (true) {
            double newTick = lFront.getCurrentPosition();


            if (newTick == oldTick) {
                break;
            }

            oldTick = newTick;

            Thread.sleep(5);
        }*/

        telemetry.addLine("done");
        telemetry.update();


    }

    public void checkArmPosition(double degreesForArm) {

    }



    public double convertMMToTicks(double targetDistanceInMM) {

        //301 = circumferance mm
        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7 / 1.4) / 301.59;

        double targetPos = targetDistanceInMM * MM_TO_TICKS;

        return targetPos;
    }

    public double getRemainingTicks(double targetDistanceInMM) {
        double targetDistanceInTicks = convertMMToTicks(targetDistanceInMM);
        double remainingDistance = targetDistanceInTicks - lFront.getCurrentPosition();
        telemetry.addLine("target"+targetDistanceInTicks);
        telemetry.addLine("remaining distance 1    "+remainingDistance);

        return remainingDistance;
    }

    public double computeDrivetrainPower(double targetDistanceInMM) {
        final double P_VALUE = 0.002;

        double remainingDistance = getRemainingTicks(targetDistanceInMM);
        if (remainingDistance < 10){
            return 0;
        }
        double proportionalPower = P_VALUE*remainingDistance;
        telemetry.addLine(String.valueOf(proportionalPower));
        return proportionalPower;
    }


    public boolean checkReachedDistance(double targetDistanceInMM, boolean servoPosition) {
        double newTicks = lFront.getCurrentPosition();

        boolean isStopped = false;
        boolean done = false;
        double remainingDistance = getRemainingTicks(targetDistanceInMM);

        telemetry.addLine("reamaining distance"+remainingDistance);
        telemetry.addLine("current pos"+lFront.getCurrentPosition());

        /*telemetry.addLine("isStopped:"+isStopped);
        telemetry.addLine("oldTick:"+oldTick);
        telemetry.addLine("newTicks"+newTicks);
        telemetry.addLine("millis:"+millis);
        telemetry.addLine("lastCheckMillis"+lastCheckMillis);*/
        if (millis > lastCheckMillis + 250) {
            lastCheckMillis = millis;
            newTicks = lFront.getCurrentPosition();
            telemetry.addLine("inside");

            if (oldTick == newTicks) {
                isStopped = true;
                telemetry.addLine("inside if 2");
            }
            oldTick = newTicks;


        }
        if (remainingDistance < 10 && isStopped) {
            arm.setPower(0);
            setMotorPower(0, 0, 0, 0);
            done = true;
        }
        telemetry.update();
        return done;
    }

    public void autoImuTurning(int degrees) {


        final double P_VALUE_FOR_TURNING_IMU = 0.002;


        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opMode.opModeIsActive() && !(-yaw < degrees + 5 && -yaw > degrees - 5)) {

            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addLine(String.valueOf(yaw));


            double angleError = degrees - yaw;

            telemetry.addLine(String.valueOf(angleError));

            lFront.setPower(P_VALUE_FOR_TURNING_IMU * angleError);
            lBack.setPower(P_VALUE_FOR_TURNING_IMU * angleError);
            rFront.setPower(-P_VALUE_FOR_TURNING_IMU * angleError);
            rBack.setPower(-P_VALUE_FOR_TURNING_IMU * angleError);

            telemetry.update();


        }
    }
}