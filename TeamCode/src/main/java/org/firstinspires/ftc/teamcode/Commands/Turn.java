package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID;

    public class Turn extends Command{
    PID PID = new PID(0.06, 0.0, 0.0);
    public DcMotor Left_Back;
    public DcMotor Right_Back;
    public DcMotor Left_Front;
    public DcMotor Right_Front;
    public double targetAngle;
    public IMU imu;
    public NavxMicroNavigationSensor navX;
    public double currentPos;
    public double PIDOutput;
    public Turn(HardwareMap hardwareMap, double targetAngle){
        this.targetAngle = targetAngle;
        PID.setSetPoint(targetAngle);
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        imu = hardwareMap.get(IMU.class, "imu");
        navX = hardwareMap.get(NavxMicroNavigationSensor.class, "NavX");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void start() {
        PID.setMaxInput(180);
        PID.setMinInput(-180);
        PID.setContinuous(true);
        PID.setMinOutput(-0.25);
        PID.setMaxOutput(0.25);
        imu.resetYaw();
 }
    public void execute() {
        /*currentPos = navX.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);*/
        currentPos = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double power = PID.updatePID(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        this.PIDOutput = power;
        System.out.println(power + "  " + PID.getError());

        Right_Front.setPower(power);
        Left_Front.setPower(power);
        Left_Back.setPower(power);
        Right_Back.setPower(-power);
    }

        public void end() {
            Left_Front.setPower(0);
            Left_Back.setPower(0);
            Right_Front.setPower(0);
            Right_Back.setPower(0);
    }

    public boolean isFinished() {
        if (!(PID.getError() == 0)) {
            if ((Math.abs(PID.getError())) < 1) {
                return true;
            }
        }
        return false;
    }
}