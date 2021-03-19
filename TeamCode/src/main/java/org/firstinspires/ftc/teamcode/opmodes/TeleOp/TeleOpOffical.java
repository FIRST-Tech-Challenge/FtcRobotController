package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.WobbleCommands;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.FlickerCommands;
import org.firstinspires.ftc.teamcode.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.commands.TransferCommands;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

@TeleOp(name = "TeleOp", group = "OpModes")
public class TeleOpOffical extends UpliftTele {

    UpliftRobot robot;
    Odometry odom;
    DriveCommands drive;
    TransferCommands transfer;
    FlickerCommands flicker;
    IntakeCommands intake;
    ShooterCommands shooter;
    WobbleCommands wobble;

    @Override
    public void initHardware() {

        robot = new UpliftRobot(this);
        if(robot.driveInitialized) {
            odom = robot.odometry;
            drive = new DriveCommands(this, robot, robot.driveSub);
            drive.enable();
        }
        if(robot.transferInitialized) {
            transfer = new TransferCommands(this, robot, robot.transferSub);
            transfer.enable();
        }
        if(robot.flickerInitialized) {
            flicker = new FlickerCommands(this, robot, robot.flickerSub);
            flicker.enable();
        }
        if(robot.intakeInitialized) {
            intake = new IntakeCommands(this, robot, robot.intakeSub);
            intake.enable();
        }
        if(robot.shooterInitialized) {
            shooter = new ShooterCommands(this, robot, robot.shooterSub);
            shooter.enable();
        }
        if(robot.wobbleInitialized) {
            wobble = new WobbleCommands(this, robot, robot.wobbleSub);
            wobble.enable();
        }

    }

    @Override
    public void initAction() {

        robot.readPositionFiles();

    }

    @Override
    public void bodyLoop() {
        robot.safeSleep(1);
    }

    @Override
    public void exit() {
        robot.clearPositionFiles();
    }
}
