package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/**
 * This routine determines the effective forward offset for the lateral tracking wheel.
 * The procedure executes a point turn at a given angle for a certain number of trials,
 * along with a specified delay in milliseconds. The purpose of this is to track the
 * change in the y position during the turn. The offset, or distance, of the lateral tracking
 * wheel from the center or rotation allows the wheel to spin during a point turn, leading
 * to an incorrect measurement for the y position. This creates an arc around around
 * the center of rotation with an arc length of change in y and a radius equal to the forward
 * offset. We can compute this offset by calculating (change in y position) / (change in heading)
 * which returns the radius if the angle (change in heading) is in radians. This is based
 * on the arc length formula of length = theta * radius.
 *
 * To run this routine, simply adjust the desired angle and specify the number of trials
 * and the desired delay. Then, run the procedure. Once it finishes, it will print the
 * average of all the calculated forward offsets derived from the calculation. This calculated
 * forward offset is then added onto the current forward offset to produce an overall estimate
 * for the forward offset. You can run this procedure as many times as necessary until a
 * satisfactory result is produced.
 */
@Config
@Disabled

@Autonomous(group="drive")
public class TrackingWheelForwardOffsetTuner extends LinearOpMode {
    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (!(drive.getLocalizer() instanceof StandardTrackingWheelLocalizer)) {
            RobotLog.setGlobalErrorMsg("StandardTrackingWheelLocalizer is not being set in the "
                    + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
                    + "(hardwareMap));\" is called in SampleMecanumDrive.java");
        }

        telemetry.addLine("Press play to begin the forward offset tuner");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        MovingStatistics forwardOffsetStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            drive.turnAsync(Math.toRadians(ANGLE));

            while (!isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double forwardOffset = StandardTrackingWheelLocalizer.FORWARD_OFFSET +
                    drive.getPoseEstimate().getY() / headingAccumulator;
            forwardOffsetStats.add(forwardOffset);

            sleep(DELAY);
        }

        telemetry.clearAll();
        telemetry.addLine("Tuning complete");
        telemetry.addLine(Misc.formatInvariant("Effective forward offset = %.2f (SE = %.3f)",
                forwardOffsetStats.getMean(),
                forwardOffsetStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
