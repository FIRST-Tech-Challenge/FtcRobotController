package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

/* Ported from Rayden's blue_down_far_load with Limelight Vision & Path branching. */
@Autonomous(name = "BlueScoring", group = "Main")
public class BlueScoring extends AutoBase {

    @Override
    protected void runRoutine() {
        double p = RobotConstants.AUTO_POWER;

        // Step 1: spin fly wheel for 3 seconds to get good power
        robot.launcher.spinUp();
        waitSeconds(2);
        // Step 2: Limelight TipDetect scan (wait up to 8s for AprilTag cluster 42..45)
        ElapsedTime waitTimer = new ElapsedTime();
        boolean isTipped = false;

        while (opModeIsActive() && waitTimer.seconds() < 8.0) {
            robot.update(); // CRITICAL: Updates Limelight camera feed and telemetry on every frame!

            isTipped = robot.vision.isHiveTipped(RobotConstants.TAG_BLUE_SCORING_MIN, RobotConstants.TAG_BLUE_SCORING_MAX);

            telemetry.addData("Scan Timer", "%.1fs remaining", 8.0 - waitTimer.seconds());
            telemetry.addData("Limelight Target", robot.vision.hasTarget() ? "YES" : "NO");
            telemetry.addData("Tip Detected", isTipped ? "TRUE" : "FALSE");

            if (isTipped) {
                break;
            }
            sleep(50);
        }

        double remainingSeconds = Math.max(0.0, 8.0 - waitTimer.seconds());

        // Branching Logic
        if (isTipped) {
            robot.intake.toggle();
            shoot(4);


            leaveAndPark();
            // Saw the camera tag! Run motors for remaining time left on the 8s timer
//            if (remainingSeconds > 0) {
//                driveSeconds(remainingSeconds, p);
//            }
            // Then execute Path A (Reverse 8 IN & run leaveAndPark)
//            driveInches(8, -p);
//            leaveAndPark();
        } else if (waitTimer.seconds() >= 8.0) {
            // 8s limit reached -> Path A: Reverse 8 IN & run leaveAndPark
            leaveAndPark();
        } else {
            // Path B: Cleared / Upright -> Reverse 19 IN and stop
            leaveAndPark();
        }
    }
    private void leaveAndPark() {
        driveInches(24, RobotConstants.AUTO_POWER);
        turnToDegrees(-90, RobotConstants.AUTO_POWER);
        driveInches(41, RobotConstants.AUTO_POWER);
    }
}