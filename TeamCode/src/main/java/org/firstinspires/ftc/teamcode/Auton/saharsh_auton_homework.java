/**
 *  Heavy inspo from:
 *  @see <a href="https://www.youtube.com/watch?v=p6g8CbNj0eM&t=1266s">
 *       The Best Way to Multitask in FTC Auto (Pedro Pathing)
 *       </a>
 */

package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Simple 3-step auton: drive to a shooting spot, shoot, then park.
 * Poses are placeholders: measure them on your field and change them.
 */
@Autonomous(name = "saharsh_auton_homework", group = "Practice")
public class saharsh_auton_homework extends OpMode {

    // Path follower and pose builder
    private Follower follower;
    private final PoseFactory pf = PoseFactory.degrees();

    // Placeholder poses (x inches, y inches, heading degrees)
    private final Pose startPose = pf.of(9, 9, 90);
    private final Pose shootPose = pf.of(50, 50, 45);
    private final Pose parkPose  = pf.of(36, 24, 90);

    // Motors and feeder servos
    private DcMotor outtake, intake, transfer;
    private CRServo leftFeed, rightFeed;

    // Paths
    private Path toShoot() { return line(startPose, shootPose).linear(startPose, shootPose); }
    private Path toPark()  { return line(shootPose, parkPose).linear(shootPose, parkPose); }

    @Override
    public void init() {
        // Bulk read mode for hub efficiency
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Capitalized names match the TeleOps
        outtake   = hardwareMap.get(DcMotor.class, "Outtake");
        intake    = hardwareMap.get(DcMotor.class, "Intake");
        transfer  = hardwareMap.get(DcMotor.class, "Transfer");
        leftFeed  = hardwareMap.get(CRServo.class, "left");
        rightFeed = hardwareMap.get(CRServo.class, "right");

        // Reset scheduler and start at startPose
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        // Run the auton steps
        schedule(routine());
    }

    // Main step-by-step auton sequence
    private Command routine() {
        return sequential(
                instant(() -> outtake.setPower(0.85)),   // spin up first
                follow(follower, toShoot()),             // drive to shooting spot
                waitMs(1000),                            // let the flywheel settle
                instant(this::feedOn),
                waitMs(4000),                            // shoot
                instant(this::allOff),
                follow(follower, toPark())               // park
        );
    }

    // Turn on feeder/intake
    private void feedOn() {
        transfer.setPower(0.8);
        intake.setPower(1.0);
        leftFeed.setPower(1.0);
        rightFeed.setPower(-1.0);
    }

    // Turn off all motors
    private void allOff() {
        outtake.setPower(0);
        transfer.setPower(0);
        intake.setPower(0);
        leftFeed.setPower(0);
        rightFeed.setPower(0);
    }

    @Override
    public void loop() {
        // Update follower and run scheduled steps
        follower.update();
        Scheduler.execute();
        telemetry.addLine("running");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Reset scheduler and turn off motors on stop
        Scheduler.reset();
        if (outtake != null) allOff();
    }
}
