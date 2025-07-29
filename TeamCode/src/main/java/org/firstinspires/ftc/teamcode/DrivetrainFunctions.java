package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Disabled
public class DrivetrainFunctions {
    public DcMotor leftFrontDrive = null;
    public double leftFrontPower = 0;

    public DcMotor leftBackDrive = null;
    public double leftBackPower = 0;

    public DcMotor rightFrontDrive = null;
    public double rightFrontPower = 0;

    public DcMotor rightBackDrive = null;
    public double rightBackPower = 0;

    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    public boolean isDisabled = false;

    private double initAttempts = 0;


    public IMU imu;
    public IMU.Parameters myIMUparameters;

    public DrivetrainFunctions(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }

    public DrivetrainFunctions(LinearOpMode l, boolean isAuto){
        linearOpMode = l;
        InitializeForAuto();
    }


    private void Initialize(){
        try {
            leftFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LF");
            leftBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LBLE");
            rightFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RFBE");
            rightBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RBRE");


//            try{
//                drive = new SampleMecanumDrive(linearOpMode.hardwareMap);
//            }catch(NullPointerException e){
//                odometryIsDisabled = true;
//            }

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //RB, RF, LB, LF

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void Reinitialize(){
        try {
            leftFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LFRE");
            leftBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LBLE");
            rightFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RFBE");
            rightBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RB");



            isDisabled = false;
        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find motors.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    private void InitializeForAuto(){
            leftFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LF");
            leftBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "LBLE");
            rightFrontDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RFBE");
            rightBackDrive = linearOpMode.hardwareMap.get(DcMotor.class, "RBRE");
            imu = linearOpMode.hardwareMap.get(IMU.class, "imu");

            myIMUparameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

    }

    public void setLeftFrontPower(double power) {
        if (Math.abs(power-leftFrontPower) > 0.02) {
            leftFrontPower = power;
            leftFrontDrive.setPower(power);
        }
    }
    public void setLeftBackPower(double power) {
        if (Math.abs(power-leftBackPower) > 0.02) {
            leftBackPower = power;
            leftBackDrive.setPower(power);
        }
    }
    public void setRightFrontPower(double power) {
        if (Math.abs(power-rightFrontPower) > 0.02) {
            rightFrontPower = power;
            rightFrontDrive.setPower(power);
        }
    }
    public void setRightBackPower(double power) {
        if (Math.abs(power-rightBackPower) > 0.02) {
            rightBackPower = power;
            rightBackDrive.setPower(power);
        }
    }

    public double getX(){
        return (leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition())/2.0;
    }

    /**
     * This code drives the robot relative to the robot itself
     * @param  x  joy stick y
     * @param  y  joy stick x
     * @param  rx  joy stick rotation
     * @param  speedScalar speed multiplier
     */
    public void Move(float y, float x, float rx, double speedScalar){ //x is forward/backward
        if(isDisabled)
            return;
        x = -x;
        y *= 1.1f;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        setLeftFrontPower(((y + x + rx) / denominator) * speedScalar);
        setLeftBackPower(((y + -x - rx) / denominator) * speedScalar);
        setRightFrontPower(((y + -x + rx) / denominator) * speedScalar);
        setRightBackPower(((y + x - rx) / denominator) * speedScalar);
    }

    /**
     * This code drives the robot relative to is position on the field
     * @param  x  joy stick y (Forward/Backward)
     * @param  y  joy stick x (Left/Right)
     * @param  rx  joy stick rotation
     * @param  speedScalar speed multiplier
     * @param  botHeading  the current heading of the robot (in radians)
     */
    public void MoveFieldOriented (float x, float y, float rx, double speedScalar, double botHeading){
        if(isDisabled)
            return;

        x = -x;

        double rotY = y * Math.cos(-botHeading) + x * Math.sin(-botHeading);
        double rotX = y * Math.sin(-botHeading) - x * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        setLeftFrontPower((rotX + rotY + rx) / denominator * speedScalar);
        setLeftBackPower((rotX - rotY + rx) / denominator * speedScalar);
        setRightFrontPower((rotX - rotY - rx) / denominator * speedScalar);
        setRightBackPower((rotX + rotY - rx) / denominator * speedScalar);
    }

    public void Stop(){
        setLeftFrontPower(0);
        setLeftBackPower(0);
        setRightFrontPower(0);
        setRightBackPower(0);
    }

    public boolean areMotorsOn() {
        if(isDisabled)
            return false;

        return rightFrontPower != 0
                || leftFrontPower != 0
                || rightBackPower != 0
                || leftBackPower != 0;
    }

    public class DriveForTimeRR implements Action {
        private boolean initialized = false;
        private double time = 0.0;
        private ElapsedTime timer = new ElapsedTime();

        private double x, y;

        public DriveForTimeRR(double targetTime, double xPower, double yPower) {
            time = targetTime;
            x = xPower;
            y = yPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            packet.addLine("In RR action");
            packet.addLine("Drive For Time");
            packet.put("Time Elapsed", timer.milliseconds());
            if (timer.milliseconds() < time) {
                Move((float)x, (float)y, 0, 1.0);
                return true;
            } else {
                Stop();
                return false;
            }
        }
    }
    public Action DriveForTimeAction(double targetTime, double xPower, double yPower) {
        return new DrivetrainFunctions.DriveForTimeRR(targetTime, xPower, yPower);
    }

    public class TurnToAngleRR implements Action {
        private boolean initialized = false;
        private double angle = 0.0;
        private ElapsedTime timer = new ElapsedTime();
        private double r = 0.0;
        private double timeout = 0.0;
        private double startAngle = 0.0;
        private double error = 0.0;

        public TurnToAngleRR(double targetAngle, double timeoutTime) {
            timeout = timeoutTime;
            angle = targetAngle;
            r = 0.0;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            packet.addLine("In RR action");
            packet.addLine("Rotate To Angle");
            packet.put("Time Elapsed", timer.milliseconds());

            error = angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            r = error / Math.PI * 6;
            if(error > Math.PI/4) r = 1;

            r = r * Math.abs(r);

            if (timer.milliseconds() < timeout /*&& error > 0.3*/) {
                setLeftFrontPower(-r);
                setLeftBackPower(-r);
                setRightFrontPower(r);
                setRightBackPower(r);

                return true;
            } else {
                Stop();
                return false;
            }
        }
    }
    public Action TurnToAngleAction(double targetAngle, double timeoutTime) {
        return new DrivetrainFunctions.TurnToAngleRR(targetAngle, timeoutTime);
    }
}
