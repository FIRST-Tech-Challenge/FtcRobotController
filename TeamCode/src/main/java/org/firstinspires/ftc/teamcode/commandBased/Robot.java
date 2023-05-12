package org.firstinspires.ftc.teamcode.commandBased;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.commands.MoveArmIncrementally;
import org.firstinspires.ftc.teamcode.commandBased.commands.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.MoveElevator;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.PointCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.rr.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.rr.util.DashboardUtil;


import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

@Config
@TeleOp(name="Command Based", group="Linear Opmode")
public class Robot extends BlackOp {

    //declare subsystem variables
    public static DrivetrainSubsystem drivetrainSS;
    public static ElevatorSubsystem elevatorSS;
    public static ArmSubsystem armSS;

    TwoWheelTrackingLocalizer localizer;


    @Override
    public void go() {

        //cancel all previous commands
        CommandScheduler.getInstance().reset();

        //create subsystem objects
        drivetrainSS = new DrivetrainSubsystem(hardwareMap);
        elevatorSS = new ElevatorSubsystem(hardwareMap);
        armSS = new ArmSubsystem(hardwareMap);

        localizer = new TwoWheelTrackingLocalizer(hardwareMap, drivetrainSS);
        localizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        //create gamepads
        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);
        ReforgedGamepad operator = new ReforgedGamepad(gamepad2);

        //create drivetrain mode commands
        FieldCentric fieldCentric = new FieldCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        RobotCentric robotCentric = new RobotCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        PointCentric pointCentric = new PointCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                Constants.TARGET,
                drivetrainSS.convertRRPose(localizer.getPoseEstimate())
        );

        //create elevator commands
        MoveElevator eleLow = new MoveElevator(elevatorSS, Constants.eleLow);
        MoveElevator eleHigh = new MoveElevator(elevatorSS, Constants.eleHigh);

        //create arm commands
        MoveArmIncrementally armForward = new MoveArmIncrementally(armSS, 10);
        MoveArmIncrementally armBackward = new MoveArmIncrementally(armSS, -10);
        MoveArmToAngle armIdle = new MoveArmToAngle(armSS, 0);

        //start robot in field-centric mode
        robotCentric.schedule();

        waitForStart();



        Scheduler.launchOnStart(this, () -> {

            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            //activate scheduler
            CommandScheduler.getInstance().run();

            localizer.update();
            Pose2d pose = localizer.getPoseEstimate();

            //drivetrain speed controls
            driver.left_bumper.onRise(() -> drivetrainSS.setSpeedMultipliers(0.5, 0.5, 0.5))
                              .onFall(() -> drivetrainSS.setSpeedMultipliers(1, 1, 1));

            //drivetrain mode controls
            driver.a.onRise(() -> {
                pointCentric.cancel();
                fieldCentric.cancel();
                robotCentric.schedule();
            });
            driver.b.onRise(() -> {
                pointCentric.cancel();
                robotCentric.cancel();
                fieldCentric.schedule();
            });
            driver.x.onRise(() -> {
                fieldCentric.cancel();
                robotCentric.cancel();
                pointCentric.schedule();
            });

            //elevator controls
            driver.dpad_up.onRise(eleHigh::schedule);
            driver.dpad_down.onRise(eleLow::schedule);

            //arm controls
            driver.dpad_right.onRise(armForward::schedule);
            driver.dpad_left.onRise(armBackward::schedule);
            driver.y.onRise(armIdle::schedule);

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, pose);

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            mTelemetry().addData("X", pose.getX());
            mTelemetry().addData("Y", pose.getY());
            mTelemetry().addData("Heading", Math.toDegrees(pose.getHeading()));
            mTelemetry().addData("Raw Gyro", Math.toDegrees(drivetrainSS.getRawExternalHeading()));
            mTelemetry().addData("heading gyro", Math.toDegrees(drivetrainSS.getHeading()));
            mTelemetry().addData("turn speed", drivetrainSS.getTurnSpeed());
            mTelemetry().addData("turn target", drivetrainSS.getTurnTarget());
            mTelemetry().update();
        });
    }
}
