package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Autonomous(name = "RedScoring", group = "Main")
public class RedScoring extends AutoBase {

    @Override
    protected void runRoutine() {
        // give the flywheel time to get up to speed
        robot.launcher.spinUp();
        waitSeconds(2);

        // only shoot if the hive is tipped
        if (scanForTip(RobotConstants.TAG_RED_SCORING_MIN, RobotConstants.TAG_RED_SCORING_MAX)) {
            shoot(4);
        }

        leaveAndPark();
    }

    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

        driveInches(24, p);
        turnToDegrees(-90, p);
        driveInches(41, p);
    }
}
