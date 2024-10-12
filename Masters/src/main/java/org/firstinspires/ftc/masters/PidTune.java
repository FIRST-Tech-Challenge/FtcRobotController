package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class PidTune extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private DcMotorEx rotateSlide;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rotateSlide = hardwareMap.get(DcMotorEx.class, "extendSlide");
        rotateSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        controller.setPID(p,i,d);
        int rotatePos = rotateSlide.getCurrentPosition();
        double pid = controller.calculate(rotatePos, target);
        double ticks_in_degree = 537.7 / 180.0;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double lift = pid + ff;

        rotateSlide.setPower(lift);

        telemetry.addData("ArmPos", rotatePos);
        telemetry.addData("Target", target);
        telemetry.update();

    }


}

