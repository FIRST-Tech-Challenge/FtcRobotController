package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class PidTune extends OpMode {

    private PIDController controller;

    public static double p = 0.00057, i = 0, d = 0.000000015;
    public static double f = 0.002;

    public static int target = 0;

    //

    DcMotor extension1, extension2;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        extension2 = hardwareMap.dcMotor.get("extension2");
        extension2.setDirection(DcMotorSimple.Direction.REVERSE);
        extension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extension1 = hardwareMap.dcMotor.get("extension1");

    }

    public void loop(){
        controller.setPID(p,i,d);
        int rotatePos = extension2.getCurrentPosition();
        double pid = controller.calculate(rotatePos, target);

        double lift = pid + f;

        extension2.setPower(lift);
        extension1.setPower(lift);

        telemetry.addData("ArmPos", rotatePos);
        telemetry.addData("Target", target);
        telemetry.update();

    }


}

