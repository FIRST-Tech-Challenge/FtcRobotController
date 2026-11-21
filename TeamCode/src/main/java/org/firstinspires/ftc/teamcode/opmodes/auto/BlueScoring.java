package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Autonomous(name = "BlueScoring", group = "Competition")
public class BlueScoring extends AutoBase {

    @Override
    protected void runRoutine() {
        // start the flywheel, then scan for the hive tags from the starting position
        robot.launcher.spinUp();

        if (scanForTip(RobotConstants.TAG_BLUE_SCORING_MIN, RobotConstants.TAG_BLUE_SCORING_MAX)) {
            shoot();
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
        driveInches(81, p);
        turnToDegrees(70, p);
        driveInches(10, p);
        driveSeconds(1.5, p);
        driveSeconds(0.2, -p);
    }
}
