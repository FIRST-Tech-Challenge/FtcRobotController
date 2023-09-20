package org.firstinspires.ftc.teamcode.TANVIII;

import static android.os.SystemClock.sleep;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Objects;

public class Robot {
    HardwareMap hwMap;

    //declare local variables for all motors
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor armMotor;
    Servo leftyServo;
    Servo rightyServo;
    double prevError = 0;
    double prevTime = 0;

    public Robot (HardwareMap hardwareMap) {
        this.hwMap = hardwareMap;

        //initialize all motors
        this.fl = hardwareMap.dcMotor.get("fl");
        this.fr = hardwareMap.dcMotor.get("fr");
        this.bl = hardwareMap.dcMotor.get("bl");
        this.br = hardwareMap.dcMotor.get("br");

        this.armMotor = hardwareMap.dcMotor.get("a");

        this.leftyServo = hardwareMap.servo.get("1");
        this.rightyServo = hardwareMap.servo.get("2");

        //reset encoder
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //zero pwr behavior (auto)
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setDrivetrainPower (double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
    public void setArmPower (double armPower) {
        armMotor.setPower(armPower);
    }
    public int getEncoderPosition (String motorName) {
        if (Objects.equals(motorName, "fl")) {
            return fl.getCurrentPosition();
        } else if (Objects.equals(motorName, "fr")) {
            return fr.getCurrentPosition();
        } else if (Objects.equals(motorName, "bl")) {
            return bl.getCurrentPosition();
        } else if (Objects.equals(motorName, "br")) {
            return br.getCurrentPosition();
        } else {
            assert false;
            return 0;
        }
    }
    public void moveArm (int ticks) {
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
        while (armMotor.isBusy()) {
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

    public void setServoPos (boolean intake) {
        if (intake) {
            leftyServo.setPosition(0);
            rightyServo.setPosition(1);
        } else {
            leftyServo.setPosition(1);
            rightyServo.setPosition(0);
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

    public double getCurrentHeading (IMU imu) {
        double currentYaw;
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return currentYaw;
    }


    public void setHeading (double wantedAbsoluteAngle, Robot robot, IMU imu, Telemetry telemetry) {
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
            robot.setHeading(179.5, robot, imu, telemetry);
        }

        double currentHeading = robot.getCurrentHeading(imu);
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


        robot.setDrivetrainPower(power, power, power, power);
        telemetry.addLine(String.valueOf(robot.getCurrentHeading(imu)));
        telemetry.addLine(String.valueOf(power));
        telemetry.update();

        prevError = error;
        prevTime = currentTime;

    }
}