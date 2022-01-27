package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "TeleOp Competition", group = "Competition")
public class TeleOpCompetition extends MasterTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();
        reset();
        waitForStart();

        while (opModeIsActive()) {
            driver1.update();
            driver2.update();

            driveRobot();
            driveLeftCarousel();
            driveRightCarousel();
            driveGrabber();
            driveArm();
            driveArmManual();

            if (driver2.getLeftTriggerValue() > 0.5 && driver2.getRightTriggerValue() > 0.5) {
                reset();
            }
        }
    }
}