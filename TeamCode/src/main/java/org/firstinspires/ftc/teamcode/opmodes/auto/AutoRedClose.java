package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

/* Ported from Rayden's red_down_close_load. */
@Autonomous(name = "Red Close", group = "Main")
public class AutoRedClose extends AutoBase {

    @Override
    protected void runRoutine() {
        // TODO: check which cell is up before shooting, e.g.
        //   boolean tipped = robot.vision.isHiveTipped(RobotConstants.TAG_RED_SCORING_MIN, RobotConstants.TAG_RED_SCORING_MAX);
        // (confirm on the robot whether SCORING or AUDIENCE is the one to check from this start position)
        leaveAndPark();      // milestone 1
        // Milestone 2: leave, get in position, shoot(4), then park.
        // TODO: skip shoot() and go park if secondsLeft() is low
    }

    // TODO: confirm this path ends in the park zone for this start position.
    private void leaveAndPark() {
        driveInches(24, RobotConstants.AUTO_POWER);
        turnToDegrees(-90, RobotConstants.AUTO_POWER);
        driveInches(41, RobotConstants.AUTO_POWER);
    }

    // TODO: drive to the park zone from wherever we are (robot.odo has the pose).
    //       Override onOverdrive() to call it so we still park if time runs out.
}
