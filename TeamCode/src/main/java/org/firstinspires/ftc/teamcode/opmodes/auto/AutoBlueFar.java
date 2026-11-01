package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

/* Ported from Rayden's blue_down_far_load. */
@Autonomous(name = "Blue Far", group = "Main")
public class AutoBlueFar extends AutoBase {

    @Override
    protected void runRoutine() {
        // TODO: check which cell is up before shooting, e.g.
        //   boolean tipped = robot.vision.isHiveTipped(RobotConstants.TAG_BLUE_SCORING_MIN, RobotConstants.TAG_BLUE_SCORING_MAX);
        // (confirm on the robot whether SCORING or AUDIENCE is the one to check from this start position)
        leaveAndPark();      // milestone 1
        // Milestone 2: leave, get in position, shoot(4), then park.
        // TODO: skip shoot() and go park if secondsLeft() is low
    }

    // TODO: confirm this path ends in the park zone for this start position.
    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

        driveInches(12, p);
        turnToDegrees(90, p);
        driveInches(30, p);
        turnToDegrees(0, p);
        driveInches(79, p);
        turnToDegrees(80, p);
        driveInches(10, p);
        driveSeconds(3.0, p);    // push into the wall to square up
        driveSeconds(0.5, -p);   // back out
    }

    // TODO: drive to the park zone from wherever we are (robot.odo has the pose).
    //       Override onOverdrive() to call it so we still park if time runs out.
}
