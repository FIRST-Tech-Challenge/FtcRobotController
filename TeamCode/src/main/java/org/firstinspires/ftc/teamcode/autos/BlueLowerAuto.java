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
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.OpModeStorage;

@Autonomous(name = "Blue Lower", group = "Autonomous")
public class BlueLowerAuto extends OpMode {

    private Follower follower;
    private IntakeSubsystem_Motor intake;
    private ShooterSubsystemFF shooter;
    private StopperSubsystem stopper;
    private VisionSubsystem vision;

    // Red AUDIENCE rotates to Blue FAR: IDs 42–45.
    private static final VisionSubsystem.Goal HIVE_GOAL =
            VisionSubsystem.Goal.FAR;

    private boolean hiveReady = false;

    private static final double SHOOT_RPM = 3200;
    private static final double DRIVE_TIMEOUT = 5.0;
    private static final double GATE_TIME = 2.0;
    private static final double PARK_DEADLINE = 25.0;

    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    private int step = 0;

    private final PoseFactory poses = PoseFactory.degrees();

    // Rotated 180 degrees:
    // X -> 144 - X, Y -> 144 - Y, heading -> heading + 180.
    private final Pose startPose =
            poses.of(82, 8, 90);

    private final Pose shootPose =
            poses.of(82, 12, 90);

    private final Pose exitPose =
            poses.of(125.0, 22.0, 25.9379);

    private Path shootPath;
    private Path parkPath;

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
        vision = new VisionSubsystem(hardwareMap);

        shooter.setTargetRpm(SHOOT_RPM);
        intake.stopIntake();
        stopper.off();

        shootPath = Paths.line(startPose, shootPose)
                .linear(startPose, shootPose);

        parkPath = Paths.line(shootPose, exitPose)
                .tangent();

        OpModeStorage.autonomousIsBlue = true;
        OpModeStorage.autonomousEndPose = null;
    }

    @Override
    public void start() {
        autoTimer.reset();

        vision.resetHiveDetection();
        hiveReady = false;

        stopper.off();

        // Intake and shooter run continuously until Stop.
        intake.startIntake();
        shooter.update(true);

        follower.follow(shootPath);
        setStep(0);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update(true);

        // true selects Blue tags.
        // Detection can latch during the approach.
        hiveReady = vision.hiveReady(HIVE_GOAL, true);

        // If shooting hasn't started by 25 seconds, skip it and park.
        if (autoTimer.seconds() >= PARK_DEADLINE
                && (step == 0 || step == 1)) {

            stopper.off();

            parkPath = Paths.line(follower.pose(), exitPose)
                    .tangent();

            follower.follow(parkPath);
            setStep(3);
        }

        switch (step) {

            // Approach shooting position.
            // The overall 25-second deadline still applies.
            case 0:
                if (!follower.isBusy()) {
                    follower.hold(shootPose);
                    setStep(1);
                }
                break;

            // Wait for both hive readiness and shooter RPM.
            case 1:
                if (hiveReady && shooter.isReady()) {
                    stopper.on();
                    setStep(2);
                }
                break;

            // Shoot for two seconds, then park.
            case 2:
                if (stepTimer.seconds() >= GATE_TIME) {
                    stopper.off();

                    follower.follow(parkPath);
                    setStep(3);
                }
                break;

            // Parking path: five-second timeout.
            case 3:
                if (driveFinished()) {
                    follower.hold(exitPose);
                    setStep(4);
                }
                break;

            // Hold final position until Stop.
            case 4:
                break;
        }

        OpModeStorage.autonomousEndPose = follower.pose();

        telemetry.addData("Auto time", "%.2f", autoTimer.seconds());

        telemetry.addData(
                "Park deadline remaining",
                "%.1f",
                Math.max(0.0, PARK_DEADLINE - autoTimer.seconds())
        );

        telemetry.addData("Step", step);
        telemetry.addData("Step time", "%.2f", stepTimer.seconds());
        telemetry.addData("Path busy", follower.isBusy());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());

        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );

        telemetry.addData("RPM", shooter.getVelocityRpm());
        telemetry.addData("Ready", shooter.isReady());

        telemetry.addData("Hive selected", "Blue %s", HIVE_GOAL);
        telemetry.addData("Hive expected IDs", "42–45");
        telemetry.addData("Hive visible now", vision.isHiveVisible());
        telemetry.addData("Hive visible IDs", vision.getHiveVisibleIds());
        telemetry.addData("Hive detection latched", vision.hasSeenHive());
        telemetry.addData(
                "Hive first seen IDs",
                vision.getHiveFirstSeenIds()
        );

        telemetry.addData(
                "Hive seconds since seen",
                "%.2f",
                vision.getHiveElapsedSeconds()
        );

        telemetry.addData(
                "Hive wait remaining",
                "%.2f",
                vision.getHiveWaitRemainingSeconds()
        );

        telemetry.addData("Hive ready", hiveReady);

        String status;

        switch (step) {
            case 0:
                status = "Driving to shoot pose";
                break;

            case 1:
                if (!vision.hasSeenHive()) {
                    status = "Waiting for Blue FAR tags 42–45";
                } else if (!hiveReady) {
                    status = "Waiting two seconds after detection";
                } else if (!shooter.isReady()) {
                    status = "Waiting for shooter RPM";
                } else {
                    status = "Ready to shoot";
                }
                break;

            case 2:
                status = "Shooting";
                break;

            case 3:
                status = "Driving to park";
                break;

            default:
                status = "Holding parking pose";
                break;
        }

        telemetry.addData("Auto status", status);
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();

        shooter.stop();
        intake.stopIntake();
        stopper.off();

        follower.stop();
        follower.update();

        if (vision != null) {
            vision.stop();
        }
    }
}