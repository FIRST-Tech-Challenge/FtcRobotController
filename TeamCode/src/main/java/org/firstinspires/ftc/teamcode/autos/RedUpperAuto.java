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

@Autonomous(name = "Red Upper", group = "Autonomous")
public class RedUpperAuto extends OpMode {

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
            poses.of(53.5, 139.3, 270);

    private final Pose shootPose =
            poses.of(53.5, 135.3, 270);

    private final Pose collectPose =
            poses.of(48, 130, 90);

    private final Pose collectControl =
            poses.of(53.3288, 109.5317, 0);

    private final Pose returnControl =
            poses.of(53.2602, 109.0498, 0);

    private final Pose exitPose =
            poses.of(14.0947, 107.6721, -154.0621);

    private Path path1;
    private Path path2;
    private Path path3;
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
        intake.stopIntake();
        stopper.off();

        path1 = Paths.line(startPose, shootPose)
                .linear(startPose, shootPose);

        path2 = Paths.curve(
                shootPose,
                collectControl,
                collectPose
        ).linear(shootPose, collectPose);

        path3 = Paths.curve(
                collectPose,
                returnControl,
                shootPose
        ).linear(collectPose, shootPose);

        path5 = Paths.line(shootPose, exitPose)
                .tangent();

        OpModeStorage.autonomousIsBlue = false;
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

            // Hold shooting position and wait for RPM.
            // No shooter readiness timeout.
            case 1:
                if (shooter.isReady()) {
                    stopper.on();
                    setStep(2);
                }
                break;

            // Shoot preloads for 1.5 seconds.
            case 2:
                if (stepTimer.seconds() >= GATE_TIME) {
                    stopper.off();

                    follower.follow(path2);
                    setStep(3);
                }
                break;

            // PATH 2: Follow the collection curve.
            // Intake stays running. Five-second timeout.
            case 3:
                if (driveFinished()) {
                    follower.follow(path3);
                    setStep(4);
                }
                break;

            // PATH 3: Return to shooting position.
            // Intake stays running. No driving timeout.
            case 4:
                if (!follower.isBusy()) {
                    follower.hold(shootPose);
                    setStep(5);
                }
                break;

            // Hold shooting position and wait for RPM.
            // No shooter readiness timeout.
            case 5:
                if (shooter.isReady()) {
                    stopper.on();
                    setStep(6);
                }
                break;

            // Shoot collected balls for 1.5 seconds.
            case 6:
                if (stepTimer.seconds() >= GATE_TIME) {
                    stopper.off();

                    follower.follow(path5);
                    setStep(7);
                }
                break;

            // PATH 5: Drive to parking position.
            // Five-second driving timeout.
            case 7:
                if (driveFinished()) {
                    follower.hold(exitPose);
                    setStep(8);
                }
                break;

            // Hold final position.
            // Intake and shooter remain running until Stop.
            case 8:
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