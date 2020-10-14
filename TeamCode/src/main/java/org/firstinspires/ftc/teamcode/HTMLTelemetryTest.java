package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HTMLTEST")
public class HTMLTelemetryTest extends LinearOpMode {
    public BetterTelemetry betterTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        betterTelemetry = new BetterTelemetry(this);
        betterTelemetry.textFormat.bold(true).fontSize(5).color(BetterTelemetry.Color.PINK);
        while (opModeIsActive()){
            betterTelemetry.addLine("hello");
            betterTelemetry.addData("foo :", gamepad1.right_trigger);
            betterTelemetry.addLine();
            betterTelemetry.addLine(BetterTelemetry.format().fontSize(7).bold(true), "big boi");
            betterTelemetry.addData(BetterTelemetry.format().color(BetterTelemetry.Color.RED), "bar", 69);
            betterTelemetry.update();

        }
    }
}
