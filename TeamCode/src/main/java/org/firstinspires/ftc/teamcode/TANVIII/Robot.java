package org.firstinspires.ftc.teamcode.TANVIII;

import static android.os.SystemClock.sleep;

import android.os.SystemClock;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Objects;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    DcMotor lFront;
    DcMotor rFront;
    DcMotor lBack;
    DcMotor rBack;
    DcMotor arm;
    IMU imu;
    double prevError = 0;
    double prevTime = 0;

    public Robot (HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        //initialize all motors
        this.lFront = hardwareMap.dcMotor.get("fl");
        this.rFront = hardwareMap.dcMotor.get("fr");
        this.lBack = hardwareMap.dcMotor.get("bl");
        this.rBack = hardwareMap.dcMotor.get("br");

        this.arm = hardwareMap.dcMotor.get("a");

        this.telemetry = telemetry;
        this.opMode = opMode;

        //reset encoder
        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //zero pwr behavior (auto)
        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        // Initialize IMU using Parameters
        imu.initialize(myIMUparameters);
    }
    public void setDrivetrainPower (double flPower, double frPower, double blPower, double brPower) {
        lFront.setPower(flPower);
        rFront.setPower(frPower);
        lBack.setPower(blPower);
        rBack.setPower(brPower);
    }
    public void setArmPower (double armPower) {
        arm.setPower(armPower);
    }
    public int getEncoderPosition (String motorName) {
        if (Objects.equals(motorName, "fl")) {
            return lFront.getCurrentPosition();
        } else if (Objects.equals(motorName, "fr")) {
            return rFront.getCurrentPosition();
        } else if (Objects.equals(motorName, "bl")) {
            return lBack.getCurrentPosition();
        } else if (Objects.equals(motorName, "br")) {
            return rBack.getCurrentPosition();
        } else {
            assert false;
            return 0;
        }
    }
    public void moveArm (int ticks) {
        arm.setTargetPosition(ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        while (arm.isBusy()) {
            sleep(10);
        }
    }
    public void setArmPos (boolean goingUp) {
        if (goingUp) {
            moveArm(2150);
        } else {
            moveArm(-2150);
        }
    }
    public double bigAbsVal (double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }
        return max;
    }
    public double getCurrentHeading () {
        double currentYaw;
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return currentYaw;
    }
    public void setHeading (double wantedAbsoluteAngle) {
        double currentTime = SystemClock.elapsedRealtimeNanos();

        //TODO: add conditionals for >180 and <179, maybe also 360
        if (wantedAbsoluteAngle < -179) {

        } else if (wantedAbsoluteAngle > 180) {
            /*
            telemetry.addLine("error: wantedAbsoluteAngle takes range -179 through 180");
            telemetry.update();
            return;
            */
        } else if (wantedAbsoluteAngle == 180) {
            setHeading(179.5);
        }

        double currentHeading = getCurrentHeading();
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        currentHeading = robotOrientation.getYaw(AngleUnit.DEGREES);
        double setTo;

        setTo = wantedAbsoluteAngle;

        double KP = 0.06; //started 0.15
        double KD = 2_500_000;

        double error = setTo - currentHeading; //error is degrees to goal
        double errorDer = (error - prevError)/(currentTime - prevTime);

        double power = (KP * error) + (KD * errorDer);
        // + kd * der

        if (Math.abs(error) < 0.1) {
            power = 0;
        }

        //cap power
        power = Range.clip(power, -1, 1);
//        if (power > 1) {
//            power = 1;
//        } else if (power < -1) {
//            power = -1;
//        }


        setDrivetrainPower(power, power, power, power);
        telemetry.addLine(String.valueOf(getCurrentHeading()));
        telemetry.addLine(String.valueOf(power));
        telemetry.update();

        prevError = error;
        prevTime = currentTime;

    }
}