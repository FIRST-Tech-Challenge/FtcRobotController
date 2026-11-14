package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Autonomous(name = "RedAudience", group = "Main")
public class RedAudience extends AutoBase {

    @Override
    protected void runRoutine() {
        // start the flywheel, then scan for the hive tags from the starting position
        robot.launcher.spinUp();

        // tags in view: shoot to tip the hive. Not in view: nothing to shoot at
        if (scanForTip(RobotConstants.TAG_RED_AUDIENCE_MIN, RobotConstants.TAG_RED_AUDIENCE_MAX)) {
            shoot(3);
        }

        leaveAndPark();
    }

    // leave and park, then square up on the wall
    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

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
