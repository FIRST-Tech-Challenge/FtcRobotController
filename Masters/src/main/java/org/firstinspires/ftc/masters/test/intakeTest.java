package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;

@Config // Enables FTC Dashboard
@TeleOp(name = "Intake Test")
public class intakeTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Init init = new Init(hardwareMap);
        Intake intake = new Intake(init, telemetry);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a) {
                intake.intakePower(.75);
            } else if (gamepad1.b){
                intake.intakePower(-.75);
            } else {
                intake.intakePower(0);
            }

        }
    }
}

