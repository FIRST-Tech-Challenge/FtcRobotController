package org.firstinspires.ftc.teamcode.methods;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class AutoCommands {
    private Launcher launcher;
    private Turret turret;
    private Intake intake;
    private Transfer transfer;
    private ShootingCoordinator shooting;
    private boolean shootRequested, collectRequested;

    // how long to sit at the flower collecting balls
    public static long FLOWER_WAIT_MS = 2000;

    LimelightVision vision;

    public AutoCommands(HardwareMap hardwareMap) {
        // One shared, rate-limited battery reading instead of per-class reads every loop.
        BatteryVoltage battery = new BatteryVoltage(hardwareMap);
        launcher = new Launcher(hardwareMap, battery);
        intake = new Intake(hardwareMap, battery);
        transfer = new Transfer(hardwareMap, battery);
        shooting = new ShootingCoordinator(launcher, transfer, intake);
        turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap, 4);
    }

    public Command shooterTogg = instant(() -> shootRequested = !shootRequested);

    public Command stopShooter = instant(() -> { shootRequested = false; shooting.stopFeed(); });

    public Command intakeTogg = instant(() -> collectRequested = true);

    public Command stopIntake = instant(() -> { collectRequested = false; intake.stop(); });

    public Command rampTogg = instant(() -> intake.toggleRamp());

    public Command rampDowner = instant(() -> intake.rampDowner());

    public Command fWheelTogg = instant(() -> launcher.toggleOn());

    public Command holdBalls = instant(() -> intake.holdBalls());

    public Command toggleAuto = instant(() -> turret.toggleTurretAuto());

    public Command startVis = instant(() -> vision.shouldStart());

    // takes the OpMode's follower (the one that gets update()d every loop)
    public Command flowerCollectClose(Follower follower) {
        return sequential(
                intakeTogg,
                rampTogg,
                follow(follower, AutoPaths.GeneralPaths.flowerCloseTo()),
                flowerCloseJiggle(follower),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                follow(follower, AutoPaths.GeneralPaths.flowerCloseFro()
                        .with(Constants.foresightConfig.maxPathSpeed.at(0.2))
                ),
                holdBalls,
                intakeTogg
        );
    }

    public Command flowerCollectFar(Follower follower) {
        return sequential(
                intakeTogg,
                follow(follower, AutoPaths.GeneralPaths.flowerFarTo()),
                Commands.lazy(() -> Commands.waitMs(FLOWER_WAIT_MS)),
                follow(follower, AutoPaths.GeneralPaths.flowerFarFro()),
                holdBalls,
                intakeTogg
        );
    }

    public Command flowerCollectFarShoot(Follower follower) {
        return sequential(
                intakeTogg,
                follow(follower, AutoPaths.GeneralPaths.flowerFarTo()),
                shooterTogg,
                Commands.waitMs(1000),
                shooterTogg,
                flowerFarJiggle(follower),
                Commands.waitMs(500),
                flowerFaHalfJiggle(follower),
                shooterTogg,
                Commands.waitMs(1000),
                follow(follower, AutoPaths.GeneralPaths.flowerFarFro())
        );
    }

    //(.)(.)    <-- titties hehe
    public Command flowerFarJiggle(Follower follower) {
        return sequential(
                follow(follower, AutoPaths.GeneralPaths.flowerJiggle()),
                follow(follower, AutoPaths.GeneralPaths.jiggleFlower())
        );
    }

    public Command flowerFaHalfJiggle(Follower follower) {
        return sequential(
                follow(follower, AutoPaths.GeneralPaths.halfJiggle())
        );
    }


    public Command flowerCloseJiggle(Follower follower) {
        return sequential(
                follow(follower, AutoPaths.GeneralPaths.flowerJiggleC()),
                follow(follower, AutoPaths.GeneralPaths.jiggleFlowerC())
        );
    }





    // Call every loop with the follower pose: runs the flywheel (distance-based speed).
    public void updateLauncher(LimelightVision.TagObservation vision, int count, double robotYawRateDegPerSec, Follower follower) {

    }

    public void otherUpdate(Follower follower) {
        launcher.updateDistanceTarget(follower.pose().x(), follower.pose().y());
        shooting.update(shootRequested, collectRequested, false);
    }

    public void updateTransfer() {
        // Compatibility with every Red/Blue auto: otherUpdate owns all motor updates.
    }

    public Launcher getLauncher() { return launcher; }
    public SynchronizedFeed.State getFeedState() { return shooting.state(); }

    /** Preserve a short full-speed pulse instead of cancelling it during readiness/ramp. */
    public Command waitForFeedMs(double milliseconds) {
        return sequential(Commands.waitUntil(() -> shooting.state() == SynchronizedFeed.State.FEED),
                Commands.waitMs(milliseconds));
    }

    public void stopAll() {
        shootRequested = collectRequested = false;
        shooting.stopAll();
        launcher.stopAll();
        intake.stop();
        transfer.stopAll();
        turret.stop();
    }

    public String getLauncherFault() {
        return launcher.getFault();
    }
}
