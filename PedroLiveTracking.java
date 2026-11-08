package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.RobotConstants;

@TeleOp(name = "Pedro Live Tracking", group = "Tracking")
public class PedroLiveTracking extends LinearOpMode {
    private Follower follower;

    @Override
    public void runOpMode() {
        // 1. Initialize the follower using RobotConstants configuration
        follower = RobotConstants.create(hardwareMap);

        // 2. Set starting position on the field (X, Y, Heading in Radians)
        // By default, X=12 inches, Y=12 inches, facing 0 degrees (forward)
        Pose startPose = new Pose(12, 12, Math.toRadians(0));
        follower.setPose(startPose);

        telemetry.addData("Status", "Initialized using RobotConstants. Place robot at (12, 12) facing forward.");
        telemetry.update();

        // Wait for the driver to press the PLAY button
        waitForStart();

        // This loop runs continuously during the match
        while (opModeIsActive() && !isStopRequested()) {
            // 3. Keep updating tracker math from dead wheel encoders + IMU
            follower.update();

            // 4. Grab the current position matrix
            Pose currentPose = follower.pose();

            // 5. Send live coordinates to the Driver Station screen
            telemetry.addData("-- Robot Position--", "");
            telemetry.addData("X Position (Inches)", currentPose.x());
            telemetry.addData("Y Position (Inches)", currentPose.y());
            telemetry.addData("Heading (Degrees)", Math.toDegrees(currentPose.heading()));
            telemetry.update();
        }
    }
}

