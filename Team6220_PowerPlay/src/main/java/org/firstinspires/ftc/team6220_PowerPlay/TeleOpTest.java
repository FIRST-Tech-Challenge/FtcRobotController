package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HolonomicTest")
public class TeleOpTest extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();
        telemetry.addLine("Waiting for start :)");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            TeleOpDrive();
        }
    }
}
