package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
        odom = robot.odometry;
        drive = new DriveCommands(this, robot, robot.driveSub);
        transfer = new TransferCommands(this, robot, robot.transferSub);
        flicker = new FlickerCommands(this, robot, robot.flickerSub);
        intake = new IntakeCommands(this, robot, robot.intakeSub);
        shooter = new ShooterCommands(this, robot, robot.shooterSub);
        intake = new IntakeCommands(this, robot, robot.intakeSub);
        wobble = new WobbleCommands(this, robot, robot.wobbleSub);

    }

    @Override
    public void initAction() {

        drive.enable();
        transfer.enable();
        flicker.enable();
        intake.enable();
        shooter.enable();
        wobble.enable();

        robot.readPositionFiles();
        wobble.wobble.openWobble();

        robot.shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5, 0, 0, 18));
        robot.shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5, 0, 0, 30));

    }

    @Override
    public void bodyLoop() {
        logData(robot);
        displayFullTelemetry(robot);
    }

    @Override
    public void exit() {
        robot.clearPositionFiles();
    }
}
