package org.firstinspires.ftc.teamcode.opMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.teamcode.bot.Bot;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;

/**
 * Everything the Pedro autos share. An auto only has to say which paths to drive, in its
 * own {@link #BuildPaths()}; this class drives them one after another, then holds the robot
 * still at the end. To write a new auto, copy {@link AutoShapeZ} and change its paths.
 *
 * <p><b>How Pedro measures</b>: inches and radians. The robot starts at {@link #StartPose()},
 * {@code (0, 0)} facing heading 0, unless the auto says otherwise. From there:
 * <ul>
 *   <li>x grows as the robot drives FORWARD
 *   <li>y grows as the robot drives LEFT
 *   <li>heading grows as the robot turns LEFT. 90 degrees is {@code Math.toRadians(90)}.
 * </ul>
 *
 * <p>How following works: {@code follower.follow(path)} starts a path. Then, every loop,
 * {@code follower.update()} reads where the robot is and sets the motor powers. When a path
 * is done, the follower stops "following" and holds the robot in place, and we start the
 * next path. See docs/decisions/0008.
 */
public abstract class PedroAuto extends LinearOpMode {

    /** The paths this auto drives, in order. */
    protected abstract List<Path> BuildPaths();

    /** Where the robot starts: (0, 0), facing forward. An auto may override this. */
    protected Pose StartPose() {
        return new Pose(0, 0, 0);
    }

    @Override
    public void runOpMode() {
        BotProperties properties = new BotProperties();
        // In an auto, Pedro Pathing drives the wheels, so driver control is switched off.
        properties.driveDisabled = true;
        // No auto uses the launcher yet.
        properties.launcherDisabled = true;

        Bot bot = new Bot(this, properties);

        if (!PedroConstants.foresightTuned) {
            // Without tuned numbers, Pedro does not know how hard to push or when to brake.
            while (!isStopRequested()) {
                telemetry.addLine("PEDRO IS NOT TUNED YET. This auto will not move.");
                telemetry.addLine("Run AutoTune, paste the results into PedroConstants,");
                telemetry.addLine("then set foresightTuned to true. See docs/decisions/0008.");
                telemetry.update();
                idle();
            }
            return;
        }

        Follower follower = bot.CreateFollower();
        follower.setPose(StartPose());
        List<Path> paths = BuildPaths();
        bot.Init();

        while (opModeInInit()) {
            bot.StartLoop();
            bot.ShowStatus();
            telemetry.addLine("Put the robot on its start spot, facing forward.");
            telemetry.addData("Paths to drive", paths.size());
            bot.EndLoop();
        }

        int pathNumber = 0;
        follower.follow(paths.get(pathNumber));

        while (opModeIsActive()) {
            bot.StartLoop();
            follower.update();

            boolean pathFinished = !follower.following();
            boolean morePathsLeft = pathNumber + 1 < paths.size();
            if (pathFinished && morePathsLeft) {
                pathNumber = pathNumber + 1;
                follower.follow(paths.get(pathNumber));
            }

            // An auto has no gamepads, so there is no HandleUserInput() here.
            bot.Process();

            Pose where = follower.pose();
            telemetry.addData("Path", (pathNumber + 1) + " of " + paths.size());
            telemetry.addData("Done", pathFinished && !morePathsLeft);
            telemetry.addData("x (forward, in)", where.x());
            telemetry.addData("y (left, in)", where.y());
            telemetry.addData("Heading (deg)", Math.toDegrees(where.heading()));
            bot.EndLoop();
        }

        follower.stop();
    }
}
