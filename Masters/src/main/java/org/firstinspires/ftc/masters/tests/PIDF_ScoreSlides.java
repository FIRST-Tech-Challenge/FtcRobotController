package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class PIDF_ScoreSlides extends OpMode {

    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.05;

    public static int target = 0;

    public final double ticks_in_degree = 384.5 / 180;

    private DcMotor backSlides;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backSlides = hardwareMap.dcMotor.get("backSlides");
        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int slidePos = backSlides.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        backSlides.setPower(power);

        telemetry.addData("pos ", slidePos);
        telemetry.addData("target ", target);
        telemetry.update();
    }

}
