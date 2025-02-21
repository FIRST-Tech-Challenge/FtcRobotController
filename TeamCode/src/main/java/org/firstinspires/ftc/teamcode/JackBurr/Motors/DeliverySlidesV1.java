package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Config
public class DeliverySlidesV1 {
    public HardwareMap hardwareMap;
    public DcMotor leftSlide;
    public DcMotor rightSlide;
    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kF = 0;
    public static double ticks_per_rev_left;
    public static double ticks_per_rev_right;
    public PIDController leftController;
    public PIDController rightController;
    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        leftSlide = hardwareMap.get(DcMotor.class, "deliverySlideL");
        rightSlide = hardwareMap.get(DcMotor.class, "deliverySlideR");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runLeftSlideToPosition(int position, double power){
        if(leftSlide.getCurrentPosition() != position) {
            leftSlide.setPower(power);
            leftSlide.setTargetPosition(position);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void runRightSlideToPosition(int position, double power){
        if(rightSlide.getCurrentPosition() != position) {
            rightSlide.setPower(power);
            rightSlide.setTargetPosition(position);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void runLeftSlideToPositionPID(int target){
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftController = new PIDController(kP, kI, kD);
        ticks_per_rev_left = leftSlide.getMotorType().getTicksPerRev();
        leftController.setPID(kP, kI, kD);
        double leftFF = Math.cos(Math.toRadians(target/ticks_per_rev_left)) * kF;
        double leftPID = leftController.calculate(leftSlide.getCurrentPosition(), target);
        double leftPower = leftPID + leftFF;
        if(getLeftSlidePosition() != target) {
            leftSlide.setPower(leftPower);
        }
    }
    public void runRightSlideToPositionPID(int target){
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightController = new PIDController(kP, kI, kD);
        ticks_per_rev_right = rightSlide.getMotorType().getTicksPerRev();
        rightController.setPID(kP, kI, kD);
        double rightFF = Math.cos(Math.toRadians(target/ticks_per_rev_right)) * kF;
        double rightPID = rightController.calculate(rightSlide.getCurrentPosition(), target);
        double rightPower = rightPID + rightFF;
        if(getRightSlidePosition() != target) {
            rightSlide.setPower(rightPower);
        }
    }


    public int getLeftSlidePosition(){
        return leftSlide.getCurrentPosition();
    }
    public int getRightSlidePosition(){
        return rightSlide.getCurrentPosition();
    }
}
