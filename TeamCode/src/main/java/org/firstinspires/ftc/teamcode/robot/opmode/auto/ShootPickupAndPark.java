package org.firstinspires.ftc.teamcode.robot.opmode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Autonomous for the mecanum StarterBot, using the same hardware and launch logic as
 * BioBuzzStarterbotTeleopMecanum:
 *
 *   1. LAUNCH: spin the launcher up for LAUNCH_SECONDS. Once it is at speed, the windmill
 *      feeds elements into it and the intake helps push them along, just like holding the
 *      right bumper in the teleop.
 *   2. PICKUP: curve from start to pickupPrep through startToPickupPrepCtrlPoint with Pedro
 *      Pathing, then drive straight from pickupPrep to pickup, running the intake the whole way.
 *      Both legs run at PICKUP_SPEED_SCALE of top speed, then the robot pauses for
 *      PICKUP_PAUSE_SECONDS at the pickup spot.
 *   3. PARK: curve from pickup to end through pickupToEndCtrlPoint, then hold that spot until
 *      the end of autonomous.
 *
 * Pedro Pathing uses the Pinpoint from Constants to know where the robot is.
 */
@Autonomous(name = "ShootPickupAndPark", group = "Compete")
public class ShootPickupAndPark extends LinearOpMode {

    // Keep these in sync with BioBuzzStarterbotTeleopMecanum
    private static final int LAUNCHER_TARGET_VELOCITY = 1250; // 2678 RPM
    private static final int LAUNCHER_MIN_VELOCITY = 1200;    // 2571 RPM
    private static final double FEED_INTAKE_POWER = 0.5;
    private static final double PICKUP_INTAKE_POWER = 1.0;

    private static final double LAUNCH_SECONDS = 7.0;

    // The pickup phase drives 40% slower than full speed (60% of the robot's top forward speed)
    // so the intake has time to grab elements, then pauses at the pickup spot before parking.
    private static final double PICKUP_SPEED_SCALE = 0.2;
    private static final double PICKUP_PAUSE_SECONDS = 1.0;

    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.0, 9.0, 270);
    private final Pose startToPickupPrepCtrlPoint = poseFactory.of(40.0, 19.0, 270);

    private final Pose pickupPrep = poseFactory.of(30.0, 7.5, 225);
    private final Pose pickup = poseFactory.of(8.5, 7.5, 180);
    private final Pose pickupToEndCtrlPoint = poseFactory.of(21.0, 50.0, 270);
    private final Pose end = poseFactory.of(8.5, 96.0, 0);

    @Override
    public void runOpMode() {
        /*
         * Pedro Pathing sets up the drive motors from Constants, so we don't touch them here.
         * The robot's starting spot is (0, 0) facing 0 degrees; everything is measured from there.
         */
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);

        intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        setFeed(0);

        telemetry.update();
        waitForStart();

        /*
         * 1. LAUNCH. The robot stays still; follower.update() keeps the position up to date.
         */
        ElapsedTime launchTimer = new ElapsedTime();
        while (opModeIsActive() && launchTimer.seconds() < LAUNCH_SECONDS) {
            follower.update();
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            boolean atSpeed = launcher.getVelocity() > LAUNCHER_MIN_VELOCITY;
            setFeed(atSpeed ? 1 : 0);

            telemetry.addData("Step", "LAUNCH (%.1f s left)", LAUNCH_SECONDS - launchTimer.seconds());
            telemetry.addData("Launcher", "%.0f / %d ticks/s %s", launcher.getVelocity(),
                    LAUNCHER_TARGET_VELOCITY, atSpeed ? "(feeding)" : "(spinning up)");
            addPoseTelemetry(follower);
            telemetry.update();
        }
        launcher.setVelocity(0);
        setFeed(0);


        /*
         * 2. PICKUP. Run the intake so it grabs elements on the way. The windmill stays off so
         *    nothing goes to the launcher. This happens in two phases:
         *      a. curve from the start to pickupPrep through startToPickupPrepCtrlPoint
         *      b. drive straight from pickupPrep in to the pickup spot
         */
        setIntake(PICKUP_INTAKE_POWER);

        double pickupSpeed = PICKUP_SPEED_SCALE * Constants.foresightConfig.maxAchievableForwardVelocity.get();
        Modifier slowForPickup = Constants.foresightConfig.maxPathSpeed.at(pickupSpeed);

        follower.follow(Paths.curve(start, startToPickupPrepCtrlPoint, pickupPrep)
                .linear(start, pickupPrep).with(slowForPickup));
        followPath(follower, "PICKUP PREP (intake running, slow)");

        follower.follow(Paths.line(pickupPrep, pickup).linear(pickupPrep, pickup).with(slowForPickup));
        followPath(follower, "PICKUP (intake running, slow)");

        // Pause at the pickup spot, still holding position and running the intake.
        ElapsedTime pauseTimer = new ElapsedTime();
        while (opModeIsActive() && pauseTimer.seconds() < PICKUP_PAUSE_SECONDS) {
            follower.update();

            telemetry.addData("Step", "PICKUP PAUSE (%.1f s left)", PICKUP_PAUSE_SECONDS - pauseTimer.seconds());
            addPoseTelemetry(follower);
            telemetry.update();
        }

        setIntake(0);

        /*
         * 3. PARK. Curve from the pickup spot to the end, then hold that spot until the end of
         *    autonomous.
         */
        follower.follow(Paths.curve(pickup, pickupToEndCtrlPoint, end).linear(pickup, end));

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("Step", follower.following() ? "PARK" : "DONE (holding position)");
            if (follower.following()) {
                telemetry.addData("Distance left (in)", "%.1f", follower.distanceToEndpoint());
            }
            addPoseTelemetry(follower);
            telemetry.update();
        }

        follower.stop();
        launcher.setVelocity(0);
        setIntake(0);
    }

    /*
     * Runs the windmill at full power to feed the launcher, and the intake at half of that to
     * push elements toward it, the same as the teleop's launch(). 0 stops them all.
     */
    private void setFeed(double windmillPower) {
        windmillServo.setPower(windmillPower);
        double intakePower = windmillPower * FEED_INTAKE_POWER;
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    /*
     * Updates the follower until it finishes the current path (or the opmode stops).
     */
    private void followPath(Follower follower, String step) {
        while (opModeIsActive() && follower.following()) {
            follower.update();

            telemetry.addData("Step", step);
            telemetry.addData("Distance left (in)", "%.1f", follower.distanceToEndpoint());
            addPoseTelemetry(follower);
            telemetry.update();
        }
    }

    /*
     * Runs the intake motor and corner servos to pull elements in, without the windmill, the
     * same as the right trigger in the teleop. 0 stops them.
     */
    private void setIntake(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    private void addPoseTelemetry(Follower follower) {
        Pose pose = follower.pose();
        telemetry.addData("Pose", "x %.1f in, y %.1f in, heading %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
