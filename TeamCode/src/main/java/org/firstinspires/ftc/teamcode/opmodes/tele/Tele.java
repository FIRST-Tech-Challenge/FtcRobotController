package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveDefault;
import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.rustlib.commandsystem.InstantCommand;
import org.rustlib.core.OpModeCore;
import org.rustlib.geometry.Pose2d;
import org.rustlib.rustboard.NoticeType;
import org.rustlib.rustboard.Rustboard;

@TeleOp(name = "TeleOp")
public class Tele extends Robot implements OpModeCore {
    public static Pose2d backdropPose = blueBackdropPose;

    @Override
    public void opModeInit() {
        if (alliance == Alliance.RED) {
            backdropPose = redBackdropPose;
        }

        drive.setDefaultCommand(new DriveDefault(drive, () -> -controller1.leftStickY.getAsDouble(), () -> controller1.leftStickX.getAsDouble(), () -> -controller1.rightStickX.getAsDouble()));
        controller1.b.and(controller1.x).and(controller1.y).onTrue(new InstantCommand(() -> drive.getOdometry().setPosition(new Pose2d())));
    }

    @Override
    public void opModeLoop() {
        telemetry.addData("heading", drive.getOdometry().getPosition().rotation.getAngleDegrees());
        Rustboard.updateTelemetryNode("slide speed", "hullo");
        Rustboard.notifyActiveClient("op mode running", NoticeType.POSITIVE);
    }
}
