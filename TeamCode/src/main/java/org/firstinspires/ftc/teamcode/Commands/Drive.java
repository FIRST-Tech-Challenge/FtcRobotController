package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID;

public class Drive extends Command {
    org.firstinspires.ftc.teamcode.PID PID = new PID(0.07, 0.0, 0.2);
    org.firstinspires.ftc.teamcode.PID DrivePID = new PID(0.03, 0.0, 0.0);

    public DcMotor Left_Back;
    public DcMotor Right_Back;
    public DcMotor Left_Front;
    public DcMotor Right_Front;
    public double targetAngle;
    public IMU imu;
    public double currentPos;
    public double PIDOutput;
    public double speed;
    public double distance;
    public double targetPos;
    public double avgEncoder;
    public double backLeft;
    public double backRight;
    public double frontLeft;
    public double frontRight;

    public Drive(HardwareMap hardwareMap, double Speed, double Distance){
        this.speed = Speed;
        this.distance = Distance;
        PID.setSetPoint(0);
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }
    public void start() {
        Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPos = distance * 300;
        DrivePID.setSetPoint(targetPos);
        PID.setMaxInput(180);
        PID.setMinInput(-180);
        PID.setContinuous(true);
        PID.setMinOutput(-1);
        PID.setMaxOutput(1);
        imu.resetYaw();
        Right_Front.setTargetPosition((int) targetPos);
        Right_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Front.setTargetPosition((int) targetPos);
        Left_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right_Back.setTargetPosition((int) targetPos);
        Right_Back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Back.setTargetPosition((int) targetPos);
        Left_Back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void execute() {
        backRight = Math.abs(Right_Back.getCurrentPosition());
        backLeft = Math.abs(Left_Back.getCurrentPosition());
        frontLeft = Math.abs(Left_Front.getCurrentPosition());
        frontRight = Math.abs(Right_Front.getCurrentPosition());
        avgEncoder = (backRight + frontLeft + frontRight) / 3;
        DrivePID.updatePID(avgEncoder);
        currentPos = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double deviation = PID.updatePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        Right_Front.setPower(-(-speed/* + deviation*/));
        Left_Front.setPower(-(speed/* - deviation*/));
        Right_Back.setPower((-speed/* - deviation*/));
        Left_Back.setPower(-(speed/* + deviation*/));
        /*
        if (currentPos >= -5 && currentPos <= 5){
            Right_Front.setPower(-speed);
            Left_Front.setPower(speed);
            Left_Back.setPower(speed);
            Right_Back.setPower(-speed);
        }*/
    }

    public void end() {
        Left_Front.setPower(0);
        Left_Back.setPower(0);
        Right_Front.setPower(0);
        Right_Back.setPower(0);
    }

    public boolean isFinished() {
        if (avgEncoder - 10>= targetPos) {
          return true;
        }
        return false;
    }
}