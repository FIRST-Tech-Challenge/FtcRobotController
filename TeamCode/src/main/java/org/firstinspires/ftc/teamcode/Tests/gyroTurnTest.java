package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3;

@TeleOp
public class gyroTurnTest extends LinearOpMode {
    CompBotV3 r = new CompBotV3();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);

        waitForStart();

        r.gyroTurn(90,0.5,telemetry);
    }
}
