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

@TeleOp(group = "Test", name = "Pid Tune Extendo")
@Config
public class PidExtendoTune extends OpMode {

    private PIDController controller;

    public static double p = 0.0045, i = 0.007, d = 0.00001;
    public static double f = 0;

    public static int target = 0;

    //

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

        intakeExtendo.setPower(pid);

        telemetry.addData("ArmPos", pos);
        telemetry.addData("Target", target);
        telemetry.update();

    }


}

