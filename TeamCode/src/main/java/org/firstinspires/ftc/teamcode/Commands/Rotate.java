package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Rotate extends Command {
    private DcMotor Left_Back;
    private DcMotor Right_Back;
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    public IMU imu;
    public double newAngle;
    double integralSum = 0;
    double Kp = 0.5;
    double Ki = 0.2;
    double Kd = 0.3;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    double radianAngle;
    double Power;
    public Rotate(HardwareMap hardwareMap, double Angle){
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        this.imu = imu;
        this.newAngle = Angle;
    }
    public double angleWrap(double radians){
        while (radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    public double PIDControl(double angle, double state){
        double referenceAngle = Math.toRadians(newAngle);
        double error = angleWrap(referenceAngle - state);
        integralSum += error + timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        this.radianAngle = angle;

        double output = (error + Kp) + (derivative + Kd) + (integralSum + Ki);
        return output;
}
    public void start(){
        imu.resetYaw();
    }
    public void execute(){
    double power = PIDControl(radianAngle, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        Left_Front.setPower(power);
        Left_Back.setPower(power);
        Right_Front.setPower(power);
        Right_Back.setPower(power);
        this.Power = power;
    }
    public void end(){
        Left_Front.setPower(0);
        Left_Back.setPower(0);
        Right_Front.setPower(0);
        Right_Back.setPower(0);
    }
    public boolean isFinished() {
        if (newAngle - 1 <= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) && imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <=  newAngle + 1 && Power <= 0.1) {
           return true;
     } else {
            return false;
        }
    }
}