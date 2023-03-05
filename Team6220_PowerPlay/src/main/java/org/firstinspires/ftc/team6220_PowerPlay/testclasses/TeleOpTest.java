package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;

//@Disabled
@TeleOp(name = "TeleOpTest")
public class TeleOpTest extends BaseTeleOp {

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            driveSlidesWithController();
            telemetry.addData("limit switch", limitSwitch.getState());
        }
    }
}
