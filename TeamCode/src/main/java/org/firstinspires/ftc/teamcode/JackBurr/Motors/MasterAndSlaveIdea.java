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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp
@Config
public class MasterAndSlaveIdea extends OpMode {
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public RobotConstantsV1 constantsV1 = new RobotConstantsV1();
    public MultipleTelemetry multipleTelemetry;
    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kF = 0;
    public static double correctionFactor = 0.0038;
    public static int leftTarget = 0;
    public static int rightTarget = 0;
    public static double ticks_per_rev;
    public PIDController controller;
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
        controller = new PIDController(kP, kI, kD);
        ticks_per_rev = leftMotor.getMotorType().getTicksPerRev();
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {
        controller.setPID(kP, kI, kD);
        double leftCurrentPos = leftMotor.getCurrentPosition();
        double rightCurrentPos = rightMotor.getCurrentPosition();
        double average = (-leftCurrentPos + rightCurrentPos) / 2;
        double pid = controller.calculate(average, leftTarget);
        double leftFF = Math.cos(Math.toRadians(leftTarget/ticks_per_rev)) * kF;
        double rightFF = Math.cos(Math.toRadians(rightTarget/ticks_per_rev)) * kF;
        double leftPower = -pid - leftFF;
        double rightPower = pid + rightFF;
        double targetError = 0;
        double error = leftCurrentPos + rightCurrentPos;
        double correction = (leftCurrentPos - rightCurrentPos) * correctionFactor;

        leftMotor.setPower(leftPower - correction);
        rightMotor.setPower(rightPower + correction);

        multipleTelemetry.addData("Average: ", average + "\n" );
        multipleTelemetry.addData("Left Position", leftCurrentPos);
        multipleTelemetry.addData("Left Target", leftTarget);
        multipleTelemetry.addData("Right Position", rightCurrentPos);
        multipleTelemetry.addData("Right Target", rightTarget);
        multipleTelemetry.addData("Target error", targetError);
        multipleTelemetry.addData("error", error);
        multipleTelemetry.update();
        if(timer.seconds() > 5){
            timer.reset();
            leftTarget = leftTarget - 50;
        }
    }
}
