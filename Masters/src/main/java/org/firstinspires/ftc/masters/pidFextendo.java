package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class pidFextendo extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static double ticks_in_degrees = (double) 8192 /360;

    public static int target = 0;
    DcMotor intakeExtendo;

    @Override
    public void init() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeExtendo = hardwareMap.dcMotor.get("intakeExtendo");
        intakeExtendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void loop(){

        controller.setPID(p,i,d);
        int pos = intakeExtendo.getCurrentPosition();
        double pid = controller.calculate(pos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double lift = pid + ff;

        intakeExtendo.setPower(lift);

        telemetry.addData("Position", pos);
        telemetry.addData("Target", target);
        telemetry.update();

    }

}

