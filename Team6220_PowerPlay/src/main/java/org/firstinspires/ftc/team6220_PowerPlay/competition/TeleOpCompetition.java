package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;

@TeleOp(name = "TeleOp Competition", group = "Competition")
public class TeleOpCompetition extends BaseTeleOp {

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            driveChassisWithController();
            driveGrabberWithController();
            driveSlidesWithController();

            telemetry.addData("slide", motorLeftSlides.getCurrentPosition());
            telemetry.update();
        }
    }
}
