package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.Input;

@Config
@TeleOp(name = "armTargetPosition")
public class armTargetPos extends LinearOpMode {


    public static int position;


    @Override
    public void runOpMode() throws InterruptedException {

        Input input = new Input(hardwareMap,false);

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        BotTelemetry.setTelemetry(telemetry, dashboardTelemetry);

        waitForStart();

        while (opModeIsActive()) {
            input.setArmPosition(position);

            BotTelemetry.addData("Actual Arm Position", input.getArmPos());
            BotTelemetry.addData("Wanted Arm Position", position);
            BotTelemetry.update();
        }
    }
}
