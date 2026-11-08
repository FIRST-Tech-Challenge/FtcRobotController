package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Autonomous(name = "BlueScoring", group = "Main")
public class BlueScoring extends AutoBase {

    @Override
    protected void runRoutine() {
        double p = RobotConstants.AUTO_POWER;

        // drive up, start the flywheel, then check the hive
        driveInches(20, p);
        robot.launcher.spinUp();

        // tipped: back up to the shooting spot and shoot. Not tipped: back up a little
        if (scanForTip(RobotConstants.TAG_BLUE_SCORING_MIN, RobotConstants.TAG_BLUE_SCORING_MAX)) {
            driveInches(19, -p);
            shoot(3);
        } else {
            driveInches(8, -p);
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
