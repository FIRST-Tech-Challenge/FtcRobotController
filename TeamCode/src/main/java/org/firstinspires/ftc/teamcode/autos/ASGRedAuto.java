package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsystem;
import org.firstinspires.ftc.teamcode.utils.OpModeStorage;

@Autonomous(name = "ASG Red", group = "Autonomous")
public class ASGRedAuto extends OpMode {

    private Follower follower;
    private IntakeSubsystem_Motor intake;
    private ShooterSubsystemFF shooter;
    private StopperSubsystem stopper;

    private static final double START_DELAY = 14.0;
    private static final double SHOOT_RPM = 3200;
    private static final double DRIVE_TIMEOUT = 5.0;
    private static final double GATE_TIME = 2.0;
    private static final double FINAL_MOVE_TIME = 25.0;

    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();
    private int step = 0;
    private boolean shooterEnabled = false;

    private final PoseFactory poses = PoseFactory.degrees();

    private final Pose startPose =
            poses.of(9, 86, 90);

    private final Pose point1 =
            poses.of(30, 66, 90);

    private final Pose point2 =
            poses.of(30, 20, 90);

    private final Pose shootPose =
            poses.of(60.85, 12, 90);

    private final Pose waitPose = poses.of(60.85, 72, 90);
    private final Pose point4 = poses.of(60.85, 119, 90);
    private final Pose exitPose = poses.of(10, 119, 180);


    private Path path1;
    private Path path2;
    private Path path3;
    private Path pathToPoint4;
    private Path path4;
    private Path path5;

    private void setStep(int nextStep) {
        step = nextStep;
        stepTimer.reset();
    }

    private boolean driveFinished() {
        return !follower.isBusy()
                || stepTimer.seconds() >= DRIVE_TIMEOUT;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        intake = new IntakeSubsystem_Motor(hardwareMap);
        shooter = new ShooterSubsystemFF(hardwareMap);
        stopper = new StopperSubsystem(hardwareMap);

        shooter.setTargetRpm(SHOOT_RPM);
        shooter.stop();
        intake.stopIntake();
        stopper.off();

        path1 = Paths.line(startPose, point1)
                .constant(point1);

        path2 = Paths.line(point1, point2)
                .reverseTangent();

        path3 = Paths.line(point2, shootPose)
                .constant(shootPose);

        // Shooting position -> waiting position.
        path4 = Paths.line(shootPose, waitPose)
                .tangent();

        // Waiting position -> original point 4.
        pathToPoint4 = Paths.line(waitPose, point4)
                .tangent();

        // Original point 4 -> parking.
        path5 = Paths.line(point4, exitPose)
                .tangent();

        OpModeStorage.autonomousIsBlue = false;
        OpModeStorage.autonomousEndPose = null;
    }

    @Override
    public void start() {
        // Wait ten seconds after pressing Start.
        // No path, intake, or shooter runs during this delay.
        shooterEnabled = false;
        shooter.stop();
        intake.stopIntake();
        stopper.off();

        autoTimer.reset();
        setStep(0);
    }

    @Override
    public void loop() {
        follower.update();

        if (shooterEnabled) {
            shooter.update(true);
        }

        switch (step) {

            // Wait for the other robot to score/tip.
            case 0:
                if (stepTimer.seconds() >= START_DELAY) {
                    intake.startIntake();

                    shooter.setTargetRpm(SHOOT_RPM);
                    shooterEnabled = true;
                    shooter.update(true);

                    follower.follow(path1);
                    setStep(1);
                }
                break;

            // PATH 1: Move diagonally to (30, 66).
            // Keep facing 90 degrees.
            case 1:
                if (driveFinished()) {
                    follower.follow(path2);
                    setStep(2);
                }
                break;

            // PATH 2: Move toward (30, 20).
            // Reverse tangent keeps the robot facing 90 degrees.
            case 2:
                if (driveFinished()) {
                    follower.follow(path3);
                    setStep(3);
                }
                break;

            // PATH 3: Approach the shooting pose.
            // No driving timeout.
            case 3:
                if (!follower.isBusy()) {
                    follower.hold(shootPose);
                    setStep(4);
                }
                break;

            // Hold shooting position and wait for RPM.
            // No readiness timeout.
            case 4:
                if (shooter.isReady()) {
                    stopper.on();
                    setStep(5);
                }
                break;

            // Shoot once for 1.5 seconds.
            case 5:
                if (stepTimer.seconds() >= GATE_TIME) {
                    stopper.off();

                    follower.follow(path4);
                    setStep(6);
                }
                break;

            // PATH 4: Move to (60.85, 72).
            case 6:
                if (driveFinished()) {
                    follower.hold(waitPose);
                    setStep(7);
                }
                break;

            // Hold here until 22 seconds after Start.
            case 7:
                if (autoTimer.seconds() >= FINAL_MOVE_TIME) {
                    follower.follow(pathToPoint4);
                    setStep(8);
                }
                break;

            // Continue straight to the original point 4.
            // No five-second timeout: finish this path before turning.
            case 8:
                if (!follower.isBusy()) {
                    follower.follow(path5);
                    setStep(9);
                }
                break;

            // Turn left and drive into parking.
            case 9:
                if (driveFinished()) {
                    follower.hold(exitPose);
                    setStep(10);
                }
                break;

            // Finished.
            case 10:
                break;
        }

        OpModeStorage.autonomousEndPose = follower.pose();

        telemetry.addData("Step", step);
        telemetry.addData("Step time", "%.2f", stepTimer.seconds());

        if (step == 0) {
            telemetry.addData(
                    "Start delay remaining",
                    "%.1f",
                    Math.max(0, START_DELAY - stepTimer.seconds())
            );
        }

        telemetry.addData("Path busy", follower.isBusy());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );
        telemetry.addData("RPM", shooter.getVelocityRpm());
        telemetry.addData("Ready", shooter.isReady());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();

        shooterEnabled = false;
        shooter.stop();
        intake.stopIntake();
        stopper.off();

        follower.stop();
        follower.update();
    }
}