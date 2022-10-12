package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HolonomicTest")
public class TeleOpTest extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        composeTelemetry();
        initHardware();
        telemetry.addLine("Waiting for start :)");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            TeleOpDrive();
            telemetry.addData("averaged y pos ls",GamePadInputAVG()[0]);
        }
    }
}
