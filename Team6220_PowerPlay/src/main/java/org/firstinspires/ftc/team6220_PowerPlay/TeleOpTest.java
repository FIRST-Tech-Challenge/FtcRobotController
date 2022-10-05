package org.firstinspires.ftc.team6220_PowerPlay;

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
