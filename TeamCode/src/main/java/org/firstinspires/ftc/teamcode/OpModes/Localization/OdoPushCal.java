package org.firstinspires.ftc.teamcode.OpModes.Localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.follower.hardware.PinpointConfig;
import org.firstinspires.ftc.teamcode.follower.hardware.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Checks that the pods measure distance correctly.
 *
 * Push the robot by hand along a straight edge, forward then left, a distance
 * you measured with a tape measure. Tiles are not exact, so don't count tiles.
 * Keep it pointing the same way and press A at the end of each push.
 *
 * Controls (gamepad 1):
 *   A = finish this push
 *   B = start over
 */
@TeleOp(name = "Odo Cal: Push", group = "Test")
public class OdoPushCal extends LinearOpMode {

    private static final double PUSH_IN = 48.0;       // the distance you really push
    private static final double FOUR_BAR_TICKS_PER_MM = 19.89436789;
    private static final double MAX_TURN_DEG = 2.0;   // redo the push if it turned more

    private enum Step { FORWARD, LEFT, DONE }

    @Override
    public void runOpMode() {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(Pose.zero());

        telemetry.addLine("Line the robot up on your edge, then press start.");
        telemetry.update();

        waitForStart();

        localizer.setPose(Pose.zero());
        Step step = Step.FORWARD;
        double forwardRead = 0, forwardSide = 0, forwardTurn = 0;
        double leftRead = 0, leftSide = 0, leftTurn = 0;
        boolean lastA = false, lastB = false;

        while (opModeIsActive()) {
            localizer.update();
            Pose pose = localizer.pose();
            double turn = Math.toDegrees(Angle.normalizeSigned(pose.heading()));

            boolean aPressed = gamepad1.a && !lastA;
            boolean bPressed = gamepad1.b && !lastB;
            lastA = gamepad1.a; lastB = gamepad1.b;

            if (bPressed) {
                step = Step.FORWARD;
                localizer.setPose(Pose.zero());
            }

            switch (step) {
                case FORWARD:
                    telemetry.addLine("Step 1: push FORWARD " + PUSH_IN + " in, then press A");
                    addLive(pose, turn);
                    if (aPressed) {
                        forwardRead = pose.x();
                        forwardSide = pose.y();
                        forwardTurn = turn;
                        localizer.setPose(Pose.zero());
                        step = Step.LEFT;
                    }
                    break;

                case LEFT:
                    telemetry.addLine("Step 2: push LEFT " + PUSH_IN + " in, then press A");
                    addLive(pose, turn);
                    if (aPressed) {
                        leftRead = pose.y();
                        leftSide = pose.x();
                        leftTurn = turn;
                        step = Step.DONE;
                    }
                    break;

                case DONE:
                    addResult("Forward", forwardRead, forwardSide, forwardTurn);
                    addResult("Left", leftRead, leftSide, leftTurn);
                    addFix(forwardRead, leftRead);
                    telemetry.addLine("B to run it again");
                    break;
            }
            telemetry.update();
        }
    }

    private void addLive(Pose pose, double turn) {
        telemetry.addData("X (in)", fmt(pose.x()));
        telemetry.addData("Y (in)", fmt(pose.y()));
        telemetry.addData("Turned (deg)", fmt(turn));
    }

    private void addResult(String name, double read, double side, double turn) {
        telemetry.addData(name + " read (in)", fmt(read) + " of " + fmt(PUSH_IN));
        telemetry.addData(name + " error", read <= 0 ? "BACKWARDS, flip this pod"
                : fmt((read / PUSH_IN - 1.0) * 100.0) + " %");
        telemetry.addData(name + " sideways (in)", fmt(side));
        telemetry.addData(name + " turned (deg)", fmt(turn)
                + (Math.abs(turn) > MAX_TURN_DEG ? "  too much, redo this push" : ""));
    }

    private void addFix(double forwardRead, double leftRead) {
        if (forwardRead <= 0 || leftRead <= 0) return;

        // Both pods share one setting on the Pinpoint, so split the difference.
        // Reading too far means too few ticks per inch, so scale up by the same ratio.
        double scale = (forwardRead + leftRead) / (2.0 * PUSH_IN);
        telemetry.addLine("If both errors match, in Constants.localizerConfig set:");
        telemetry.addLine("c.encoderResolutionUnit.set(DistanceUnit.MM);");
        telemetry.addLine(String.format("c.ticksPerUnit.set(OptionalDouble.of(%.5f));",
                currentTicksPerMm() * scale));
    }

    private static double currentTicksPerMm() {
        PinpointConfig config = Constants.localizerConfig;
        if (!config.ticksPerUnit.get().isPresent()) return FOUR_BAR_TICKS_PER_MM;
        return config.ticksPerUnit.get().getAsDouble() / config.encoderResolutionUnit.get().toMm(1.0);
    }

    private static String fmt(double v) {
        return String.format("%.2f", v);
    }
}
