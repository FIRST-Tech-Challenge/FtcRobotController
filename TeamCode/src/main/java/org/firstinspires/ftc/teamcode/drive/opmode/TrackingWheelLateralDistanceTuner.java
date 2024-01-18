package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/**
 * Opmode designed to assist the user in tuning the `StandardTrackingWheelLocalizer`'s
 * LATERAL_DISTANCE value. The LATERAL_DISTANCE is the center-to-center distance of the parallel
 * wheels.
 *
 * Tuning Routine:
 *
 * 1. Set the LATERAL_DISTANCE value in StandardTrackingWheelLocalizer.java to the physical
 * measured value. This need only be an estimated value as you will be tuning it anyways.
 *
 * 2. Make a mark on the bot (with a piece of tape or sharpie or however you wish) and make an
 * similar mark right below the indicator on your bot. This will be your reference point to
 * ensure you've turned exactly 360°.
 *
 * 3. Although not entirely necessary, having the bot's pose being drawn in dashbooard does help
 * identify discrepancies in the LATERAL_DISTANCE value. To access the dashboard,
 * connect your computer to the RC's WiFi network. In your browser, navigate to
 * https://192.168.49.1:8080/dash if you're using the RC phone or https://192.168.43.1:8080/dash
 * if you are using the Control Hub.
 * Ensure the field is showing (select the field view in top right of the page).
 *
 * 4. Press play to begin the tuning routine.
 *
 * 5. Use the right joystick on gamepad 1 to turn the bot counterclockwise.
 *
 * 6. Spin the bot 10 times, counterclockwise. Make sure to keep track of these turns.
 *
 * 7. Once the bot has finished spinning 10 times, press A to finishing the routine. The indicators
 * on the bot and on the ground you created earlier should be lined up.
 *
 * 8. Your effective LATERAL_DISTANCE will be given. Stick this value into your
 * StandardTrackingWheelLocalizer.java class.
 *
 * 9. If this value is incorrect, run the routine again while adjusting the LATERAL_DISTANCE value
 * yourself. Read the heading output and follow the advice stated in the note below to manually
 * nudge the values yourself.
 *
 * Note:
 * It helps to pay attention to how the pose on the field is drawn in dashboard. A blue circle with
 * a line from the circumference to the center should be present, representing the bot. The line
 * indicates forward. If your LATERAL_DISTANCE value is tuned currently, the pose drawn in
 * dashboard should keep track with the pose of your actual bot. If the drawn bot turns slower than
 * the actual bot, the LATERAL_DISTANCE should be decreased. If the drawn bot turns faster than the
 * actual bot, the LATERAL_DISTANCE should be increased.
 *
 * If your drawn bot oscillates around a point in dashboard, don't worry. This is because the
 * position of the perpendicular wheel isn't perfectly set and causes a discrepancy in the
 * effective center of rotation. You can ignore this effect. The center of rotation will be offset
 * slightly but your heading will still be fine. This does not affect your overall tracking
 * precision. The heading should still line up.
 */
@Config
@Disabled

@TeleOp(group = "drive")
public class TrackingWheelLateralDistanceTuner extends LinearOpMode {
    public static int NUM_TURNS = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (!(drive.getLocalizer() instanceof StandardTrackingWheelLocalizer)) {
            RobotLog.setGlobalErrorMsg("StandardTrackingWheelLocalizer is not being set in the "
                    + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
                    + "(hardwareMap));\" is called in SampleMecanumDrive.java");
        }

        telemetry.addLine("Prior to beginning the routine, please read the directions "
                + "located in the comments of the opmode file.");
        telemetry.addLine("Press play to begin the tuning routine.");
        telemetry.addLine("");
        telemetry.addLine("Press Y/△ to stop the routine.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.update();

        double headingAccumulator = 0;
        double lastHeading = 0;

        boolean tuningFinished = false;

        while (!isStopRequested() && !tuningFinished) {
            Pose2d vel = new Pose2d(0, 0, -gamepad1.right_stick_x);
            drive.setDrivePower(vel);

            drive.update();

            double heading = drive.getPoseEstimate().getHeading();
            double deltaHeading = heading - lastHeading;

            headingAccumulator += Angle.normDelta(deltaHeading);
            lastHeading = heading;

            telemetry.clearAll();
            telemetry.addLine("Total Heading (deg): " + Math.toDegrees(headingAccumulator));
            telemetry.addLine("Raw Heading (deg): " + Math.toDegrees(heading));
            telemetry.addLine();
            telemetry.addLine("Press Y/△ to conclude routine");
            telemetry.update();

            if (gamepad1.y)
                tuningFinished = true;
        }

        telemetry.clearAll();
        telemetry.addLine("Localizer's total heading: " + Math.toDegrees(headingAccumulator) + "°");
        telemetry.addLine("Effective LATERAL_DISTANCE: " +
                (headingAccumulator / (NUM_TURNS * Math.PI * 2)) * StandardTrackingWheelLocalizer.LATERAL_DISTANCE);

        telemetry.update();

        while (!isStopRequested()) idle();
    }
}