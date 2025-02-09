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

@TeleOp
@Config
public class PIDTutorial extends OpMode {
    public DcMotorEx motor;
    public MultipleTelemetry multipleTelemetry;
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kF = 0;
    public static double choice = 0;
    public static int target = 500;
    public static double ticks_per_rev;
    public PIDController controller;
    public ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "intakeSlides");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controller = new PIDController(kP, kI, kD);
        ticks_per_rev = motor.getMotorType().getTicksPerRev();
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {
        controller.setPID(kP, kI, kD);
        int currentPos = motor.getCurrentPosition();
        double pid = controller.calculate(currentPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_per_rev)) * kF;
        double power = pid + ff;
        motor.setPower(power);
        multipleTelemetry.addData("Position", currentPos);
        multipleTelemetry.addData("Target", target);
        multipleTelemetry.update();
        if(timer.seconds() > 4){
            target = target + 1100;
            timer.reset();
        }
    }
}
