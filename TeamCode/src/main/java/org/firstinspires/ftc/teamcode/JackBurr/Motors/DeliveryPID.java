package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;

@TeleOp
@Config
public class DeliveryPID extends OpMode {
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public RobotConstantsV1 constantsV1 = new RobotConstantsV1();
    public MultipleTelemetry multipleTelemetry;
    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kF = 0;
    public static double choice = 0;
    public static int leftTarget;
    public static int rightTarget;
    public static double ticks_per_rev;
    public PIDController leftController;
    public PIDController rightController;
    public ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "deliverySlideL");
        rightMotor = hardwareMap.get(DcMotorEx.class, "deliverySlideR");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTarget = constantsV1.LEFT_SLIDE_HIGH_BASKET;
        rightTarget = constantsV1.RIGHT_SLIDE_HIGH_BASKET;
        leftController = new PIDController(kP, kI, kD);
        rightController = new PIDController(kP, kI, kD);
        ticks_per_rev = leftMotor.getMotorType().getTicksPerRev();
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {
        leftController.setPID(kP, kI, kD);
        rightController.setPID(kP, kI, kD);
        double leftCurrentPos = leftMotor.getCurrentPosition();
        double rightCurrentPos = rightMotor.getCurrentPosition();
        double leftPID = leftController.calculate(leftCurrentPos, leftTarget);
        double rightPID = rightController.calculate(rightCurrentPos, rightTarget);
        double leftFF = Math.cos(Math.toRadians(leftTarget/ticks_per_rev)) * kF;
        double rightFF = Math.cos(Math.toRadians(rightTarget/ticks_per_rev)) * kF;
        double leftPower = leftPID + leftFF;
        double rightPower = rightPID + rightFF;
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        multipleTelemetry.addData("Left Position", leftCurrentPos);
        multipleTelemetry.addData("Left Target", leftTarget);
        multipleTelemetry.addData("Right Position", leftCurrentPos);
        multipleTelemetry.addData("Right Target", leftTarget);
        multipleTelemetry.update();
    }
}
