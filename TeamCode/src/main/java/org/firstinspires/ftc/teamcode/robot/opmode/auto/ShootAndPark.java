package org.firstinspires.ftc.teamcode.robot.opmode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.config.LeftServoMemory;

/*
 * Autonomous for the mecanum StarterBot, using the same hardware and launch logic as
 * BioBuzzStarterbotTeleopMecanum:
 *
 *   1. LAUNCH: spin the launcher up for LAUNCH_SECONDS. Once it is at speed, the windmill
 *      feeds elements into it and the intake helps push them along, just like holding the
 *      right bumper in the teleop.
 *   2. STRAFE: drive STRAFE_INCHES to the right with Pedro Pathing, keeping the robot
 *      pointed the same way, then hold that spot until the end of autonomous.
 *
 * Pedro Pathing uses the Pinpoint from Constants to know where the robot is. Pedro measures
 * y to the robot's LEFT, so driving right means going to a negative y.
 */
@Autonomous(name = "ShootAndPark", group = "Compete")
public class ShootAndPark extends LinearOpMode {

    // Keep these in sync with BioBuzzStarterbotTeleopMecanum
    private static final int LAUNCHER_TARGET_VELOCITY = 1250; // 2678 RPM
    private static final int LAUNCHER_MIN_VELOCITY = 1200;    // 2571 RPM
    private static final double FEED_INTAKE_POWER = 0.5;

    private static final double LAUNCH_SECONDS = 7.0;

    private DcMotor intake;
    private Servo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.0, 9.0, 270);
    private final Pose end = poseFactory.of(9.0, 96.0, 270);

    private final Pose centerCtrlPoint = poseFactory.of(25.0, 39.0, 270);


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
        leftIntakeServo = hardwareMap.get(Servo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setPosition(LeftServoMemory.position());
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


        follower.follow(Paths.curve(start, centerCtrlPoint, end).linear(start, end));

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("Step", follower.following() ? "STRAFE" : "DONE (holding position)");
            if (follower.following()) {
                telemetry.addData("Distance left (in)", "%.1f", follower.distanceToEndpoint());
            }
            addPoseTelemetry(follower);
            telemetry.update();
        }

        follower.stop();
        launcher.setVelocity(0);
    }

    /*
     * Runs the windmill at full power to feed the launcher, and the intake at half of that to
     * push elements toward it, the same as the teleop's launch(). 0 stops them all.
     */
    private void setFeed(double windmillPower) {
        windmillServo.setPower(windmillPower);
        double intakePower = windmillPower * FEED_INTAKE_POWER;
        intake.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    private void addPoseTelemetry(Follower follower) {
        Pose pose = follower.pose();
        telemetry.addData("Pose", "x %.1f in, y %.1f in, heading %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
