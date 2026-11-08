package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Simple BlueAudience", group = "Competition")
public class SimpleBlueAudience extends SimpleAutoBase {

    @Override
    protected void runRoutine() {
        double p = DRIVE_POWER;

        // give the flywheel time to get up to speed
        spinUp();
        waitSeconds(2);

        // only shoot if the hive is tipped
        if (scanForTip(38, 41)) {
            shoot(4);
        }

        // leave and park
        driveInches(24, p);
        turnToDegrees(-90, p);
        driveInches(41, p);
    }
}
