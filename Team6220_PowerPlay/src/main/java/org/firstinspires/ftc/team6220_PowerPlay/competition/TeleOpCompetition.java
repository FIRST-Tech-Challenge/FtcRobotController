package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends BaseTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            driveFieldCentric();
            driveGrabberWithController();
            driveSlidesWithController();
            driveLEDs();
            resetIMU();
            slideOverride();
            telemetry.addData("isLimitSwitchPressed", limitSwitch.getState());
            telemetry.update();
        }
    }
}
