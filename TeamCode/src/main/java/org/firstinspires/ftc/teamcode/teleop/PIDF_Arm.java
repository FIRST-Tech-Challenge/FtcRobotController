package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp

public class PIDF_Arm extends OpMode {

    private PIDController controller;

    public static double p = 0.01, i = 0.25, d = 0.001;
    public static double f = 0.4;
    public static double maxSpeed = 0.7;

    public static int target = 0;

    private final double ticks_in_degrees = 1425.1 / 360.0;

    private DcMotorEx intakeArmMotor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int intakeArmPos = intakeArmMotor.getCurrentPosition();
        double pid = controller.calculate(intakeArmPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees) - Math.toRadians(50) + Math.toRadians(90)) * f;

        double power = pid + ff;
        intakeArmMotor.setPower(limitPower(power));

        telemetry.addData("pos", intakeArmPos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.update();
    }

    private double limitPower(double power){
        if (power > maxSpeed){
            return maxSpeed;
        }
        if (power < -maxSpeed){
            return -maxSpeed;
        }
        return power;
    }
}

