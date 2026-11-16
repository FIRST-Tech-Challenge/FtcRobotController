package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import org.firstinspires.ftc.teamcode.methods.LimelightVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.AutoCommands;
import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.PathVisualizer;
import org.firstinspires.ftc.teamcode.methods.Transfer;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.methods.CoordinateAim;
import org.firstinspires.ftc.teamcode.methods.CoordinateTurretControl;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue1TipFar", group="Blue")
public class Blue1TipFar extends OpMode {

    //private PathVisualizer visualizer;

    private Follower follower;
    private AutoCommands autoCommand;
    private LimelightVision vision;
    private Turret turret;
    private DcMotorEx turretMotor;
    private int turretZeroTicks;
    private Launcher launcher;
    private Servo led;
    //private Transfer transfer;

    boolean started = false;

    private double kP = 0.02;
    private double kD = 0.0;

    private final ElapsedTime yawTimer = new ElapsedTime();
    private boolean haveLastHeading = false;

    private double lastHeadingRad = 0;
    private double robotYawRateDegPerSec = 0;

    private double yawRateDegPerSec;

    private void readYaw(double headingRad) {
        double dt = yawTimer.seconds();
        yawTimer.reset();
        yawRateDegPerSec = Double.NaN;
        if (haveLastHeading && Double.isFinite(headingRad)
                && dt >= 0.005 && dt <= 0.25) {
            double delta = Math.atan2(
                    Math.sin(headingRad - lastHeadingRad),
                    Math.cos(headingRad - lastHeadingRad));
            yawRateDegPerSec = Math.toDegrees(delta / dt);
        }
        lastHeadingRad = headingRad;
        haveLastHeading = Double.isFinite(headingRad);
    }

    private Command autoRoutine() {
        return sequential(
                autoCommand.toggleAuto,
                autoCommand.fWheelTogg,
                follow(follower, AutoPaths.Opp.startToShoot()),
                Commands.waitMs(5000),
                autoCommand.shooterTogg,
                autoCommand.intakeTogg,
                Commands.waitMs(1500),
                autoCommand.intakeTogg,
                autoCommand.shooterTogg,
                autoCommand.startVis,
                autoCommand.toggleAuto,
                follow(follower, AutoPaths.Opp.shootToFlowerWait()),
                autoCommand.rampTogg,
                autoCommand.flowerCollectFarShoot(follower),
                follow(
                        follower, AutoPaths.Opp.flowerWaitToCollect()
                                .with(Constants.foresightConfig.maxPathSpeed.at(0.4))
                ),
                autoCommand.rampTogg,
                follow(follower, AutoPaths.Opp.collectCreep()),
                follow(
                        follower, AutoPaths.Opp.park()
                                .with(Constants.foresightConfig.maxPathSpeed.at(0.8))
                )
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.update();
        autoCommand = new AutoCommands(hardwareMap);
        turret = new Turret(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "TurretRot");
        launcher = autoCommand.getLauncher();
        led = hardwareMap.get(Servo.class, "led");

        turret.stopRotation();
        // This auto begins with the turret physically forward-zero. Preserve that raw
        // encoder reference across the autonomous-to-TeleOp OpMode transition.
        turretZeroTicks = turretMotor.getCurrentPosition();
        OpModeStorage.turretForwardZeroTicks = turretZeroTicks;
        OpModeStorage.turretForwardZeroConfirmed = true;

        vision = new LimelightVision(hardwareMap, 5);
        follower.setPose(AutoPaths.GeneralPoints.startFar);

       // transfer = new Transfer(hardwareMap);



        /*visualizer = new PathVisualizer(
                AutoPaths.Opp.startToShoot(),
                AutoPaths.Opp.shootToFlowerWait(),
                AutoPaths.Opp.flowerWaitToCollect(),
                AutoPaths.Opp.collectCreep(),
                AutoPaths.Opp.park()
        );*/

        //visualizer.drawNow(follower.pose());
    }
@Override
    public void start() {
        //vision.start();
        schedule(autoRoutine());
        turret.mainPD.reset();
        haveLastHeading = false;
        yawTimer.reset();

        //Turret.ROT_MAX_POWER = 0.35;
        //Turret.ROBOT_YAW_FF_GAIN = 0.002;

        //visualizer.startRecording(follower.pose());
    }

    @Override
    public void loop() {
        Scheduler.execute();
        //turret.update();
        //transfer.update();
        turret.update();
        follower.update();

        telemetry.addData("Auto?", turret.isTurretAuto());

        turret.mainPD.updateRobotTranslationFromPose(
                follower.pose().x(), follower.pose().y(), follower.pose().heading());

        //visualizer.recordPose(follower.pose());
        //visualizer.draw(follower.pose());

        readYaw(follower.pose().heading());
        vision.update();

        if (LimelightVision.shouldStart && !started) {
            vision.start();
            started = true;
        }

        telemetry.addData("Started Vision?", LimelightVision.shouldStart);


        /*if (!launcher.getFault().isEmpty()) {
            intake.stop();
        }*/

        //follower.pose().heading()


        CoordinateAim.Result aim = CoordinateAim.calculate(
                follower.pose().x(), follower.pose().y(), follower.pose().heading(),
                62.4, 82.3);
        double turretAngleDeg = (turretMotor.getCurrentPosition() - (double) turretZeroTicks)
                / CoordinateTurretControl.TICKS_PER_DEGREE;
        CoordinateTurretControl.Result control = CoordinateTurretControl.calculate(
                aim.relativeBearingDeg, turretAngleDeg, 1, 0.04);
        if (aim.valid && control.reachable) {
            double power = CoordinateTurretControl.motorPower(turretAngleDeg, control.power, -1);
            turret.setRotationPower(power);
        } else {
            turret.stopRotation();
        }
        telemetry.addData("Coordinate target bearing", aim.relativeBearingDeg);
        telemetry.addData("Coordinate turret error", control.errorDeg);

        autoCommand.otherUpdate(follower);

        autoCommand.updateTransfer();

        led.setPosition(Math.random());

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("LL Tags", vision.getVisibleTagIds());
        telemetry.addData("LL Frame Age (ms)", vision.getFrameAgeMs());
        telemetry.addData("Current Velo", launcher.getFlywheelVelo());
        telemetry.addData("Target Velo", launcher.getTargetVelocity());
        telemetry.addData("Shooting feed state", autoCommand.getFeedState());

        turret.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        //turret.stop();
        vision.stop();
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "BLUE";

        autoCommand.stopAll();
    }
}
