package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {


    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    DcMotor fLeft;
    DcMotor fRight;
    DcMotor bLeft;
    DcMotor bRight;
    DcMotor arm;
    IMU imu;

    double yaw;

    double oldTick;
    long millis = System.currentTimeMillis();
    long lastCheckMillis = System.currentTimeMillis();

    //CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
        setUpDrivetrainMotors();
        setUpImu();
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
    public void setUpDrivetrainMotors() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");


        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private double maxAbsValueDouble(double[] values) {

        double max = -Double.MIN_VALUE;


        for (double value : values) {
            if (Math.abs(value) > Math.abs(max)) {
                max = value;
            }

        }

        return Math.abs(max);
    }

    private double[] scalePowers(double[] powers) {
        double maxPower = maxAbsValueDouble(powers);

        if (maxPower < 1) {
            return powers;
        }

        return new double[]{
                powers[0] / maxPower,
                powers[1] / maxPower,
                powers[2] / maxPower,
                powers[3] / maxPower
        };
    }


    public double calculateImuPower(int degrees) {


        final double P_VALUE_FOR_TURNING_IMU = 0.002;

        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double proportionalPowerForImu = 0;

        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addLine(String.valueOf(yaw));

        double angleError = degrees - yaw;

        proportionalPowerForImu = P_VALUE_FOR_TURNING_IMU * angleError;

        telemetry.addLine(String.valueOf(angleError));


        return proportionalPowerForImu;

    } //IMU
    public void setUpArmMotor() {
        arm = hardwareMap.dcMotor.get("arm");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double calculateArmPower(int targetAngleInDegrees) {

        final double P_VALUE = 0.006;

        double remainingDistance = getRemainingTicksForArm(targetAngleInDegrees);
        if (remainingDistance < 10){
            return 0;
        }
        double proportionalPower = P_VALUE*remainingDistance;
        telemetry.addLine(String.valueOf(proportionalPower));
        return proportionalPower;

    }
    public void setArmPower(double armPower) {
        arm.setPower(armPower);
    }
    public boolean checkArmPos(int targetAngleInDegrees) {
        double newTicks = arm.getCurrentPosition();

        boolean isStopped = false;
        boolean doneArm = false;
        double remainingDistance = getRemainingTicksForArm(targetAngleInDegrees);

        telemetry.addLine("remaining distance"+remainingDistance);
        telemetry.addLine("current pos for arm"+arm.getCurrentPosition());

        /*telemetry.addLine("isStopped:"+isStopped);
        telemetry.addLine("oldTick:"+oldTick);
        telemetry.addLine("newTicks"+newTicks);
        telemetry.addLine("millis:"+millis);
        telemetry.addLine("lastCheckMillis"+lastCheckMillis);*/
        if (millis > lastCheckMillis + 250) {
            lastCheckMillis = millis;
            newTicks = arm.getCurrentPosition();
            //telemetry.addLine("inside");

            if (oldTick == newTicks) {
                isStopped = true;
                //telemetry.addLine("inside if 2");
            }
            oldTick = newTicks;


        }
        if (remainingDistance < 10 && isStopped) {
            setArmPower(0);
            doneArm = true;
        }
        return doneArm;
    }
    public double getRemainingTicksForArm(double targetDistanceInDegrees) {
        double targetDistanceInTicks = convertDegreesToTicks(targetDistanceInDegrees);
        double remainingDistance = targetDistanceInTicks - arm.getCurrentPosition();
        telemetry.addLine("target              "+targetDistanceInTicks);
        telemetry.addLine("remaining distance 1    "+remainingDistance);

        return remainingDistance;
    }
    public double convertDegreesToTicks(double targetDistanceInDegrees) {

        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        final double Degrees_TO_TICKS = 537.7 / 15;

        double targetPos = targetDistanceInDegrees * Degrees_TO_TICKS;

        return targetPos;
    }//Arm
    public void setMotorPower(double lFront, double rFront, double lBack, double rBack) {
        this.fLeft.setPower(lFront);
        this.fRight.setPower(rFront);
        this.bLeft.setPower(lBack);
        this.bRight.setPower(rBack);
    }

    public void setMotorPower(double[] powers) {
        this.fLeft.setPower(powers[0]);
        this.fRight.setPower(powers[1]);
        this.bLeft.setPower(powers[2]);
        this.bRight.setPower(powers[3]);
    }

    public double convertMMToTicks(double targetDistanceInMM) {

        //301 = circumferance mm
        //537.7, ticks per motor revolution
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7 / 301.59)*2;

        double targetPos = targetDistanceInMM * MM_TO_TICKS;

        return targetPos;
    }

    public boolean checkReachedDistance(double targetDistanceInMM) {

        boolean isStopped = false;
        boolean done = false;
        double remainingDistance = Math.abs(getRemainingTicksForDrivetrain(targetDistanceInMM));

        telemetry.addData("remainig distance for ffowrard and backeward", remainingDistance);

        if (remainingDistance < 30) {
            setMotorPower(0, 0, 0, 0);
            done = true;
        }
        return done;
    }
    public double getRemainingTicksForDrivetrain(double targetDistanceInMM) {
        double targetDistanceInTicks = convertMMToTicks(targetDistanceInMM);
        double remainingDistance = targetDistanceInTicks - fLeft.getCurrentPosition();

        return remainingDistance;
    }


    public void resetEncoder() {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double[] calculateDrivetrainPower(double targetDistanceInMM) {
        final double P_VALUE = 0.002;

        if (checkReachedDistance(targetDistanceInMM)){
            return new double[] {0, 0, 0, 0};
        }

        double remainingDistance = getRemainingTicksForDrivetrain(targetDistanceInMM);

        double proportionalPower = P_VALUE*remainingDistance;
        double scaleImu = 0;
        //telemetry.addLine("POWER  "+proportionalPower);
        return scalePowers(new double[]{
                proportionalPower + calculateImuPower(0)*scaleImu,
                proportionalPower - calculateImuPower(0)*scaleImu,
                proportionalPower + calculateImuPower(0)*scaleImu,
                proportionalPower - calculateImuPower(0)*scaleImu
        });
    }
    public void autoForward(double targetDistanceInMM) throws InterruptedException {


        final double P_VALUE = 0.006;

        //301 = circumferance mm
        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7 / 1.4) / 301.59;

        double targetPos = targetDistanceInMM * MM_TO_TICKS + fLeft.getCurrentPosition();
        double error = targetPos - fLeft.getCurrentPosition();

        final double PROPORTIONAL_POWER = P_VALUE * error;



        long lastCheckMillis = System.currentTimeMillis();
        long millis = System.currentTimeMillis();

        double oldTick = fLeft.getCurrentPosition();

        boolean isStopped = false;

        while (!(error <= 10 && isStopped) && opMode.opModeIsActive()) {

            millis = System.currentTimeMillis();

            //telemetry.addLine(String.valueOf(targetPos));
            //telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));

            error = targetPos - fLeft.getCurrentPosition();
            //telemetry.addLine(String.valueOf(error));

            fLeft.setPower(PROPORTIONAL_POWER);
            bLeft.setPower(PROPORTIONAL_POWER);
            fRight.setPower(PROPORTIONAL_POWER);
            bRight.setPower(PROPORTIONAL_POWER);

            //telemetry.addLine("moving");


            if (millis > lastCheckMillis + 500) {
                lastCheckMillis = millis;
                double newTicks = fLeft.getCurrentPosition();

                if (oldTick == newTicks) {
                    isStopped = true;
                }

                oldTick = newTicks;


            }

        }

        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);

        //telemetry.addLine(String.valueOf(lFront.getCurrentPosition() / MM_TO_TICKS));

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

    }//Auto Forward but Better
    public double convertMMToTicksForMecanum(double targetDistanceInMM) {

        //301 = circumferance mm
        //537.7, ticks per motor revolution
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7 / 301.59)*1.2;

        double targetPos = targetDistanceInMM * MM_TO_TICKS;

        return targetPos;
    }
    public double getRemainingTicksForDrivetrainMecanum(double targetDistanceInMM) {
        double targetDistanceInTicks = convertMMToTicksForMecanum(targetDistanceInMM);
        double remainingDistance = targetDistanceInTicks - fLeft.getCurrentPosition();

        return remainingDistance;
    }

    //TODO document for notebook

    public boolean checkReachedDistanceForMecanum(double targetDistanceInMM) {

        boolean done = false;
        double remainingDistance = getRemainingTicksForDrivetrain(targetDistanceInMM);

        telemetry.addData("remaining distance in ticks for mecananamasm", remainingDistance);


        if (remainingDistance < 30 && -yaw <5 && -yaw > - 5) {
            setMotorPower(0, 0, 0, 0);
            done = true;
        }
        return done;
    }
    public double[] calculateMecanumPower(double targetDistanceInMM) {
        final double P_VALUE_FOR_MECANUM = 0.002;

        if (checkReachedDistanceForMecanum(targetDistanceInMM)){
            return new double[] {0, 0, 0, 0};
        }

        double remainingDistance = getRemainingTicksForDrivetrainMecanum(targetDistanceInMM);
        double proportionalPower = P_VALUE_FOR_MECANUM*remainingDistance;

        double scaleImu = 8.15;
        //telemetry.addLine("POWER  "+ proportionalPower);
        return scalePowers(new double[]{
                + proportionalPower + calculateImuPower(0)*scaleImu,
                - proportionalPower - calculateImuPower(0)*scaleImu,
                - proportionalPower + calculateImuPower(0)*scaleImu,
                + proportionalPower - calculateImuPower(0)*scaleImu
        });
    }

    public void autoMecanuming(double targetMecanumDistance) {
        final double P_VALUE = 0.004;

        //301 = circumferance mm
        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        final double MM_TO_TICKS = 537.7/301.59;

        double targetPos = targetMecanumDistance * MM_TO_TICKS*(1.225);
        double error = targetPos - fLeft.getCurrentPosition();

        final double PROPORTIONAL_POWER = P_VALUE * error;


        long lastCheckMillis = System.currentTimeMillis();
        long millis = System.currentTimeMillis();

        double oldTick = fLeft.getCurrentPosition();

        boolean isStopped = false;


        millis = System.currentTimeMillis();

        //telemetry.addLine(String.valueOf(targetPos));
        //telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));

        error = targetPos - fLeft.getCurrentPosition();
        //telemetry.addLine(String.valueOf(error));

        double imuPower = calculateImuPower(0);

        setMotorPower(PROPORTIONAL_POWER+imuPower, -PROPORTIONAL_POWER+imuPower, -PROPORTIONAL_POWER+imuPower, PROPORTIONAL_POWER+imuPower);


        //telemetry.addLine("moving");


        if (millis > lastCheckMillis + 500) {
            lastCheckMillis = millis;
            double newTicks = fLeft.getCurrentPosition();

            if (oldTick == newTicks) {
                isStopped = true;
            }

            oldTick = newTicks;

        }



        //telemetry.addLine(String.valueOf(lFront.getCurrentPosition() / MM_TO_TICKS));



        telemetry.addLine("done");


    }


}
