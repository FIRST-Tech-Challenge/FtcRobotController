package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config // Enables FTC Dashboard
@TeleOp(name = "WhatsExtendo?")
public class whatsExtendo extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    DcMotor intakeExtendo;

    public void runOpMode() throws InterruptedException {

        intakeExtendo= hardwareMap.dcMotor.get("intakeExtendo");
        intakeExtendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeExtendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Position", intakeExtendo.getCurrentPosition());
            telemetry.update();

        }
    }
}

