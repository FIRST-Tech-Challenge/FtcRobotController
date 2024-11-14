package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDFSlideTuner extends OpMode {

    private PIDController controller;

    public static double p = 0.01, i = 0.25, d = 0.001;
    public static double f = 0;
    public static double maxSpeed = 0.5;

    public static int target = 0;

    private final double ticks_in_degrees = 537.7 / 360.0;

    private DcMotorEx rightSlideMotor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int intakeArmPos = rightSlideMotor.getCurrentPosition();
        double pid = controller.calculate(intakeArmPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;

        rightSlideMotor.setPower(limitPower(power));

        telemetry.addData("pos", intakeArmPos);
        telemetry.addData("target", target);
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

