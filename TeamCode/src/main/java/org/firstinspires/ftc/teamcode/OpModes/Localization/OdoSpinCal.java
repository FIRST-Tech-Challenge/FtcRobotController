package org.firstinspires.ftc.teamcode.OpModes.Localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.hardware.Mecanum;
import org.firstinspires.ftc.teamcode.follower.hardware.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Checks the pod offsets. The robot spins itself in place, so give it room.
 *
 * With correct offsets X and Y barely move during a spin. Wrong offsets make the
 * reported position trace a circle, and that circle says how far off they are.
 * It uses every sample over several turns, which beats one 180 turned by hand.
 *
 * Put a tape mark under a corner before starting. When the spin ends the robot
 * should be pointing the same way it started. If it isn't, the heading is off.
 */
@TeleOp(name = "Odo Cal: Spin", group = "Test")
public class OdoSpinCal extends LinearOpMode {

    private static final double TURNS = 5.0;
    private static final double POWER = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Mecanum drive = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        localizer.setPose(Pose.zero());

        telemetry.addLine("Stand clear, robot spins " + TURNS + " times after start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        localizer.setPose(Pose.zero());
        sleep(100);
        localizer.update();

        List<double[]> samples = new ArrayList<>();
        double totalTurn = 0;
        double prevHeading = localizer.pose().heading();

        drive.drive(new DrivePowers(0, 0, POWER), false);

        while (opModeIsActive() && Math.abs(totalTurn) < TURNS * 2.0 * Math.PI) {
            localizer.update();
            Pose pose = localizer.pose();

            totalTurn += Angle.normalizeSigned(pose.heading() - prevHeading);
            prevHeading = pose.heading();
            samples.add(new double[]{pose.x(), pose.y(), pose.heading()});

            telemetry.addData("Turns", fmt(totalTurn / (2.0 * Math.PI)));
            telemetry.addData("X (in)", fmt(pose.x()));
            telemetry.addData("Y (in)", fmt(pose.y()));
            telemetry.update();
        }

        drive.stop(true);
        sleep(1000);
        localizer.update();
        Pose end = localizer.pose();

        // Fit x = cx + dx*cos(h) - dy*sin(h), y = cy + dx*sin(h) + dy*cos(h).
        // (dx, dy) is how far the tracked point is from the real center of
        // rotation, in robot coordinates: forward and left.
        double meanX = 0, meanY = 0, meanCos = 0, meanSin = 0;
        for (double[] s : samples) {
            meanX += s[0];
            meanY += s[1];
            meanCos += Math.cos(s[2]);
            meanSin += Math.sin(s[2]);
        }
        int n = samples.size();
        meanX /= n; meanY /= n; meanCos /= n; meanSin /= n;

        double sumDx = 0, sumDy = 0, sumWeight = 0;
        for (double[] s : samples) {
            double x = s[0] - meanX;
            double y = s[1] - meanY;
            double c = Math.cos(s[2]) - meanCos;
            double sin = Math.sin(s[2]) - meanSin;
            sumDx += c * x + sin * y;
            sumDy += c * y - sin * x;
            sumWeight += c * c + sin * sin;
        }
        double dx = sumDx / sumWeight;
        double dy = sumDy / sumWeight;

        // What's left after the fit is the robot wandering, not offset error.
        double sumResidual = 0;
        for (double[] s : samples) {
            double x = s[0] - meanX;
            double y = s[1] - meanY;
            double c = Math.cos(s[2]) - meanCos;
            double sin = Math.sin(s[2]) - meanSin;
            sumResidual += Math.pow(x - (dx * c - dy * sin), 2) + Math.pow(y - (dx * sin + dy * c), 2);
        }
        double wander = Math.sqrt(sumResidual / n);

        // The X pod offset is its distance left of center, the Y pod offset is its
        // distance forward of center, so forward error goes to the Y pod and left to X.
        double newXPod = Constants.localizerConfig.xPodOffset.get() + dy;
        double newYPod = Constants.localizerConfig.yPodOffset.get() + dx;

        while (opModeIsActive()) {
            telemetry.addData("Offset error (in)", fmt(Math.hypot(dx, dy)));
            telemetry.addData("Robot wander (in)", fmt(wander) + "   lower is more trustworthy");
            telemetry.addData("Samples", n);
            telemetry.addData("End X / Y (in)", fmt(end.x()) + " / " + fmt(end.y()));
            telemetry.addData("End heading (deg)", fmt(Math.toDegrees(Angle.normalizeSigned(end.heading())))
                    + "   compare to the real robot");
            telemetry.addLine("In Constants.localizerConfig set:");
            telemetry.addLine(String.format("c.xPodOffset.set(%.3f);", newXPod));
            telemetry.addLine(String.format("c.yPodOffset.set(%.3f);", newYPod));
            telemetry.update();
        }
    }

    private static String fmt(double v) {
        return String.format("%.2f", v);
    }
}
