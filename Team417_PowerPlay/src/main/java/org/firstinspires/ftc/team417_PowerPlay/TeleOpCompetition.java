package org.firstinspires.ftc.team417_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="Teleop Competition")
public class TeleOpCompetition extends BaseTeleOp{

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addLine("Ready for start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            driveUsingControllers();
            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
            telemetry.addData("Front Encoder", frontEncoder.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
