package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Simple RedScoring", group = "Competition")
public class SimpleRedScoring extends SimpleAutoBase {

    @Override
    protected void runRoutine() {
        double p = DRIVE_POWER;

        // give the flywheel time to get up to speed
        spinUp();
        waitSeconds(2);

        // only shoot if the hive is tipped
        if (scanForTip(30, 33)) {
            shoot(4);
        }

        // leave and park
        //tipped up
        driveInches(24, p);
        turnToDegrees(-90, p);
        driveInches(41, p);
    }
}
