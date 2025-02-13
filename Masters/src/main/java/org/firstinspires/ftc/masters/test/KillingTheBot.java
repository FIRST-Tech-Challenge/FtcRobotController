package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outtake;

@Config // Enables FTC Dashboard
@TeleOp(name = "Bot Kill")
public class KillingTheBot extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double Blank = 0;

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Init init = new Init(hardwareMap);
        Outtake outtake = new Outtake(init, telemetry);

        VoltageSensor voltageSensor;
        voltageSensor = init.getVoltageSensor();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                outtake.setTarget(-400);
            }

            if (voltageSensor.getVoltage() < 10){
                outtake.setTarget(2500);
            }

            telemetry.addData("Volt", voltageSensor.getVoltage());
            telemetry.update();

            outtake.update();


        }
    }
}

