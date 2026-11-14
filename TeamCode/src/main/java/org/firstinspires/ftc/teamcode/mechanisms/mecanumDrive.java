package org.firstinspires.ftc.teamcode.mechanisms;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class mecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;



    public void init(HardwareMap hwMap){
        frontLeftMotor = hwMap.get(DcMotor.class, "motorFL");
        backLeftMotor = hwMap.get(DcMotor.class, "motorBL");
        frontRightMotor = hwMap.get(DcMotor.class, "motorFR");
        backRightMotor = hwMap.get(DcMotor.class, "motorBR");

        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);

        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(RevOrientation));

    }
    public void drive(double forward,double strafe, double rotate ) {
        double frontLeftPower=forward+strafe+rotate;
        double backLeftPower=forward-strafe+rotate;
        double frontRightPower=forward-strafe-rotate;
        double backRightPower=forward+strafe-rotate;

        double maxPower=1.0;
        double maxSpeed=1.0;

        maxPower=Math.max(maxPower,Math.abs(frontLeftPower));
        maxPower=Math.max(maxPower,Math.abs(backLeftPower));
        maxPower=Math.max(maxPower,Math.abs(frontRightPower));
        maxPower=Math.max(maxPower,Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed*frontLeftPower/maxPower);
        backLeftMotor.setPower(maxSpeed*backLeftPower/maxPower);
        frontRightMotor.setPower(maxSpeed*frontRightPower/maxPower);
        backRightMotor.setPower(maxSpeed*backRightPower/maxPower);


    }

    public void driveField(double forward,double strafe, double rotate) {
        double theta = Math.atan2(forward,strafe);
        double r=Math.hypot(strafe,forward);

        theta = AngleUnit.normalizeRadians(theta-
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r *Math.sin(theta);
        double newStrafe = r*Math.cos(theta);

        this.drive(newForward,newStrafe,rotate);
    }

    public void resetYaw()
    {
        imu.resetYaw();
    }

    public double getIMUYaw()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getIMURoll()
    {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public double getIMUPitch()
    {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }
}