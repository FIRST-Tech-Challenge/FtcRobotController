package org.firstinspires.ftc.teamcode.BioBuzz;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;
import org.firstinspires.ftc.teamcode.Utils.MiscUtils;
import org.firstinspires.ftc.teamcode.Utils.Movement.MovementUtils;
import org.firstinspires.ftc.teamcode.Utils.Positioning.DistanceToHive;
import org.firstinspires.ftc.teamcode.Utils.Shooting.Turret;
import org.firstinspires.ftc.teamcode.Utils.Shooting.TurretConfigurator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoDriveMain", group = "Autonomous")
public class AutoDriveMain extends LinearOpMode2026 {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57, 9, 180);
    private final Pose point1 = poseFactory.of(9.75, 9.5, 180);
    private final Pose point2 = poseFactory.of(64.9433, 131.1091, -90.8781);
    private final Pose point2Control1 = poseFactory.of(30.0, 29.9094, 0);
    private final Pose point2Control2 = poseFactory.of(0, 125.7339, 0);
    private final Pose point2Control3 = poseFactory.of(64.5333, 104.3615, 0);
    private final Pose point2Segment1Heading = poseFactory.of(64.9433, 131.1091, 180);
    private final Pose point3 = poseFactory.of(10, 106.6598, -178.837);
    private final Pose point3Control1 = poseFactory.of(64.8796, 98.922, 0);
    private final Pose point3Control2 = poseFactory.of(42.9879, 107.3295, 0);
    private final Pose path4 = poseFactory.of(57, 10, -178.9092);
    private final Pose path4Control1 = poseFactory.of(28.8077, 106.8628, 0);
    private final Pose path4Control2 = poseFactory.of(44.3732, 62.4997, 0);
    private final Pose path4Control3 = poseFactory.of(7.1052, 9.05, 0);
    private final Pose path5 = poseFactory.of(60.5, 132, -90);
    private final Pose path5Control1 = poseFactory.of(60.8783, 65.7508, 0);
    private final Pose path5Segment1Heading = poseFactory.of(60.5, 132, 180);
    private final Pose path5Segment3Start = poseFactory.of(60.5, 132, -89.8912);
    private final Pose path5Segment3End = poseFactory.of(60.5, 132, 270);
//    private static final double HIVE_TY_THRESHOLD = 5.0;

    public Turret turretAimer;
    DistanceToHive distanceToHive;

    boolean isBackAtStart = false;
    boolean notBailed = true;

    TurretConfigurator turretConfigurator;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                waitMs(500),
                shoot(),
                follow(follower, corner()),
                waitMs(2000),
                goBackIfHiveIsNotDown(),
                waitMs(500),
                conditional( () -> isBackAtStart,
                        follow(follower, startToFlower()),
                        follow(follower, crossField())
                ),
                turnIntakeOff(),
//                waitForLimelight(),
                waitMs(100),
                waitForLimelightTx(),
                shoot(),
                follow(follower, park()),
                conditional( () -> !isCompetition, //when thing is set in config to return home
                        follow(follower, toStart()),
                        instant(() -> {}) // runs if the condition is false (does nothing)
                ),
                instant(() -> {
                    MovementUtils.autoEndPose = follower.pose();
                })
        );
    }

    @Override
    public void runOpMode() {
        config();
        turretAimer = new Turret(turret, posGet, MiscUtils.getMotorPositionFromDegrees(90, 537.7));
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        turretConfigurator = new TurretConfigurator(this);
        distanceToHive = new DistanceToHive(posGet);

        waitForStart();
        long start = System.currentTimeMillis();
        flywheel.setVelocity(2200 * TICKS_PER_REV / 60.0); // 2400 was really good
        hood.setPosition(0.2);
        intake.setPower(0);

        while (Math.abs(flywheel.getVelocity() - (2100 * TICKS_PER_REV / 60.0)) > 80){
            telemetry.addData("Flywheel velocity", flywheel.getVelocity());
            telemetry.addData("Wanted velocity", (2100 * TICKS_PER_REV / 60.0));
            DoTurret();

            telemetry.update();
        }

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("ty", limelight.getLatestResult().getTy());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            if(System.currentTimeMillis()-start > 25000 && notBailed) {
                bail();
            }

            turretConfigurator.runTurret(distanceToHive.GetDistance());

            DoTurret();

            telemetry.update();
        }
    }

    public Command turnIntakeOff(){
        return instant(() -> intake.setPower(0));
    }

    public Path corner() {
        return Paths.path(Paths.line(start, point1).constant(point1))
                .with(Constants.foresightConfig.maxPathSpeed.at(.5))
                .with(Constants.foresightConfig.maxDecelerationConstraint.at(.5));
    }
    public Path crossField() {
        return Paths.path(Paths.curve(point1, point2Control1, point2Control2, point2Control3, point2).heading(Interpolator.tangent.reverse()));
    }
    public Path park() {
        return Paths.path(Paths.curve(point2, point3Control1, point3Control2, point3).tangent());
    }
    public Path toStart() {
        return Paths.curve(point3, path4Control1, path4Control2, path4Control3, path4).reverseTangent();
    }
    public Path startToFlower() {
        return Paths.curve(start, path5Control1, path5).heading(Interpolator.piecewise().until(0.01, Interpolator.constant(path5Segment1Heading)).until(0.9361, Interpolator.tangent.reverse()).until(1, Interpolator.linear(path5Segment3Start, path5Segment3End)));
    }
    public Command shoot(){
        return  sequential(
            instant(() -> intake.setPower(INTAKE_POWER)),
            instant(() -> lift.setPower(LIFT_POWER)),
            waitMs(1500),
            instant(() -> lift.setPower(0))
        );
    }

    public Path backToStart(){
        return Paths.path(Paths.line(point1, start).constant(start));
    }

//    public Command waitForLimelight() {
//        return run(() -> {
//            DoTurret();
//        }).until(() -> {
//            LLResult result = limelight.getLatestResult();
//            return result != null && result.isValid();
//        });
//    }

    public void DoTurret(){
        // This will turn the turret to the april tags (if it sees it)

        double tx = turretAimer.UpdateTxValue(limelight);

        turretAimer.ChangeAngleSetPosition(tx, posGet.getVr());

        double positionChange = MiscUtils.getMotorPositionFromDegrees(tx, 537.7);

        telemetry.addLine("tx: " + tx);
        telemetry.addLine("Amount of Position changes: " + positionChange);

        turretAimer.CheckPositionAndSetPower(telemetry);
    }

    public Command waitForLimelightTx() {
        // Keeps updating the turret position while waiting
        return infinite(this::DoTurret).until(() -> {
            double tx = turretAimer.UpdateTxValue(limelight);
            LLResult result = limelight.getLatestResult();

            DoTurret();

            // Ensure the result is valid and tx is within ±0.5 degrees of center
            return result != null && result.isValid() && Math.abs(tx) < 0.5;
        });
    }

    public void bail() {
        Scheduler.reset();
        follower.stop();
        Scheduler.schedule(
                instant(() -> {
                    MovementUtils.autoEndPose = follower.pose();
                }),
                follow(follower,park())
                );
        notBailed = false;
    }

    public Command goBackIfHiveIsNotDown() {
        return sequential(
                follow(follower, backToStart()),
                waitForLimelightTx(),
                conditional(() -> limelight.getLatestResult().isValid(), shoot(), instant(() -> {isBackAtStart = true;}))
        );


//        long startTime = System.currentTimeMillis();
//        int[] validFrames = {0};
//        boolean[] shouldGoBack = {false};
//        double timeoutMs = 800.0; // Check for up to 0.8 seconds
//
//        // Step 1: Observation phase to check for the April tag stably
//        Command observation = infinite(() -> {
//            LLResult result = limelight.getLatestResult();
//            if (result != null && result.isValid()) {
//                validFrames[0]++;
//                telemetry.addData("Valid Tag Frames", validFrames[0]);
//            }
//            DoTurret(); // Keep updating turret while looking
//        }).until(() -> {
//            if (validFrames[0] >= 7) { // Seen stably for at least 7 frames
//                shouldGoBack[0] = true;
//                return true;
//            }
//            if (System.currentTimeMillis() - startTime > timeoutMs) {
//                return true; // Timeout reached without stable detection
//            }
//            return false;
//        });
//
//        // Step 2: Sequentially run observation, then conditionally go back and shoot if seen
//        return sequential(
//                observation,
//                conditional(
//                        () -> shouldGoBack[0],
//                        sequential(
//                                follow(follower, backToStart()),
//                                shoot()
//                        ),
//                        instant(() -> {}) // Do nothing if tag wasn't seen, proceed with routine
//                )
//        );
    }
}
