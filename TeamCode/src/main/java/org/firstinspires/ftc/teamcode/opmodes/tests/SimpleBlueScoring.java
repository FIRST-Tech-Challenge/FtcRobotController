package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Simple BlueScoring", group = "Competition")
public class SimpleBlueScoring extends SimpleAutoBase {

    @Override
    protected void runRoutine() {
        double p = DRIVE_POWER;

        // drive up, start the flywheel, then check the hive
        driveInches(20, p);
        spinUp();

        // tipped: back up to the shooting spot and shoot. Not tipped: back up a little in
        if (scanForTip(42, 45)) {
            driveInches(19, -p);
            shoot(3);
        } else {
            driveInches(8, -p);
        }

        // leave and park, then square up on the wall
        driveInches(12, p);
        turnToDegrees(90, p);
        driveInches(30, p);
        turnToDegrees(0, p);
        driveInches(79, p);
        turnToDegrees(80, p);
        driveInches(10, p);
        driveSeconds(1.5, p);
        driveSeconds(0.3, -p);
    }
}
