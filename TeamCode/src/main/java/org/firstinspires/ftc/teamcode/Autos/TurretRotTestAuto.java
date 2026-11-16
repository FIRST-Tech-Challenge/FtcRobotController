package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.methods.AutoCommands;
import org.firstinspires.ftc.teamcode.methods.AutoPaths;
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.OpModeStorage;
import org.firstinspires.ftc.teamcode.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class TurretRotTestAuto extends OpMode {

    private Follower follower;
    private AutoCommands autoCommand;
    private Launcher launcher;
    private Intake intake;
    private Turret turret;

    private Command autoRoutine() {
        return sequential(
                follow(follower, AutoPaths.GeneralPaths.startToShootClose()),
                Commands.waitMs(4000),
                follow(follower, AutoPaths.Tip.shootCloseToCorner()),
                follow(follower, AutoPaths.Tip.cornerToFarShoot()),
                Commands.waitMs(4000)
                );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.GeneralPoints.startClose);
        follower.update();
        launcher    = new Launcher(hardwareMap);
        intake      = new Intake(hardwareMap);
        turret      = new Turret(hardwareMap);

    }
@Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        turret.update();

        follower.update();
        Scheduler.execute();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Turret Rot", turret.getRotationDeg());
        telemetry.addLine("I <3 Femboys");
        turret.addTelemetry(telemetry);
        telemetry.update();

    }

    @Override
    public void stop() {
        turret.stop();
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        OpModeStorage.LastAllegiance = "RED";
    }
}