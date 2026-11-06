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

@Autonomous(name = "Blue Upper", group = "Autonomous")
public class BlueUpperAuto extends OpMode {

    private Follower follower;
    private IntakeSubsystem_Motor intake;
    private ShooterSubsystemFF shooter;
    private StopperSubsystem stopper;

    private static final double SHOOT_RPM = 3200;
    private static final double DRIVE_TIMEOUT = 5.0;
    private static final double GATE_TIME = 1.5;

    private final ElapsedTime stepTimer = new ElapsedTime();
    private int step = 0;

    private final PoseFactory poses = PoseFactory.degrees();

    private final Pose startPose =
            poses.of(77.5, 138, 270);

    private final Pose shootPose =
            poses.of(77.5, 132, 270);

    private final Pose point2 =
            poses.of(136, 114, 90);

    private final Pose point3 =
            poses.of(136, 132, 90);

    private final Pose point4 =
            poses.of(117.0828, 121.9339, 270);

    private final Pose point6 =
            poses.of(110, 103.5715, 322.8751);

    private final Pose point7 =
            poses.of(115, 50, 279.8429);

    private Path path1;
    private Path path2;
    private Path path3;
    private Path path4;
    private Path path5;
    private Path path6;
    private Path path7;

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
        intake.stopIntake();
        stopper.off();

        // Initial shooting approach.
        path1 = Paths.line(startPose, shootPose)
                .tangent();

        // Move to the collection approach while turning.
        path2 = Paths.line(shootPose, point2)
                .linear(shootPose, point2);

        // Collect while facing 90 degrees.
        path3 = Paths.line(point2, point3)
                .constant(point3);

        // First return segment: turn back to 270 degrees.
        path4 = Paths.line(point3, point4)
                .linear(point3, point4);

        // Finish returning to the shooting position.
        path5 = Paths.line(point4, shootPose)
                .constant(shootPose);

        // Parking approach.
        path6 = Paths.line(shootPose, point6)
                .tangent();

        // Final parking segment.
        path7 = Paths.line(point6, point7)
                .tangent();

        OpModeStorage.autonomousIsBlue = true;
        OpModeStorage.autonomousEndPose = null;
    }

    @Override
    public void start() {
        stopper.off();

        // Intake and shooter run continuously until Stop.
        intake.startIntake();
        shooter.update(true);

        follower.follow(path1);
        setStep(0);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update(true);

        switch (step) {

            // PATH 1: Approach shooting position.
            // No driving timeout.
            case 0:
                if (!follower.isBusy()) {
                    follower.hold(shootPose);
                    setStep(1);
                }
                break;

            // Hold position and wait for shooter readiness.
            case 1:
                if (shooter.isReady()) {
                    stopper.on();
                    setStep(2);
                }
                break;

            // Shoot preloads.
            case 2:
                if (stepTimer.seconds() >= GATE_TIME) {
                    stopper.off();

                    follower.follow(path2);
                    setStep(3);
                }
                break;

            // PATH 2: Move to (141, 114), turning toward 90 degrees.
            // Five-second timeout.
            case 3:
                if (driveFinished()) {
                    follower.follow(path3);
                    setStep(4);
                }
                break;

            // PATH 3: Collect toward (141, 130), facing 90 degrees.
            // Five-second timeout.
            case 4:
                if (driveFinished()) {
                    follower.follow(path4);
                    setStep(5);
                }
                break;

            // PATH 4: Return through (117.0828, 121.9339).
            // No driving timeout.
            case 5:
                if (!follower.isBusy()) {
                    follower.follow(path5);
                    setStep(6);
                }
                break;

            // PATH 5: Finish returning to shooting position.
            // No driving timeout.
            case 6:
                if (!follower.isBusy()) {
                    follower.hold(shootPose);
                    setStep(7);
                }
                break;

            // Hold position and wait for shooter readiness.
            case 7:
                if (shooter.isReady()) {
                    stopper.on();
                    setStep(8);
                }
                break;

            // Shoot collected balls.
            case 8:
                if (stepTimer.seconds() >= GATE_TIME) {
                    stopper.off();

                    follower.follow(path6);
                    setStep(9);
                }
                break;

            // PATH 6: First parking movement.
            // Five-second timeout.
            case 9:
                if (driveFinished()) {
                    follower.follow(path7);
                    setStep(10);
                }
                break;

            // PATH 7: Final parking movement.
            // Five-second timeout.
            case 10:
                if (driveFinished()) {
                    follower.hold(point7);
                    setStep(11);
                }
                break;

            // Hold final position.
            // Intake and shooter remain running until Stop.
            case 11:
                break;
        }

        OpModeStorage.autonomousEndPose = follower.pose();

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
    }
}