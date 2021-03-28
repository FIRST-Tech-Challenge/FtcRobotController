package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.InstantCommand;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.EncodedMotorGroup;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.logger.Log;
import com.technototes.logger.LogConfig;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OperatorInterface;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterSetFlapCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterSetSpeedCommand;

@TeleOp(name = "shooterconfig")
public class ShooterConfigOpMode extends CommandOpMode implements Loggable {
    @LogConfig.Disabled
    public Robot robot;

    @Override
    public void uponInit() {
        robot = new Robot();
        driverGamepad.dpadUp.whenPressed(new ShooterSetSpeedCommand(robot.shooterSubsystem, ()->robot.shooterSubsystem.getVelocity()+0.05));
        driverGamepad.dpadDown.whenPressed(new ShooterSetSpeedCommand(robot.shooterSubsystem, ()->robot.shooterSubsystem.getVelocity()-0.05));
        driverGamepad.dpadLeft.whenPressed(new ShooterSetFlapCommand(robot.shooterSubsystem, ()->robot.shooterSubsystem.getFlapPosition()+0.05));
        driverGamepad.dpadRight.whenPressed(new ShooterSetFlapCommand(robot.shooterSubsystem, ()->robot.shooterSubsystem.getFlapPosition()-0.05));
        driverGamepad.a.whilePressed(new DriveCommand(robot.drivebaseSubsystem, driverGamepad.leftStick, driverGamepad.rightStick));
        driverGamepad.leftBumper.whenPressed(new InstantCommand(()->robot.drivebaseSubsystem.turn(robot.drivebaseSubsystem.getExternalHeading()-0.1)));
        driverGamepad.rightBumper.whenPressed(new InstantCommand(()->robot.drivebaseSubsystem.turn(robot.drivebaseSubsystem.getExternalHeading()+0.1)));

    }

    @Log.Number(name = "x", index = 0)
    public double x(){
        return robot.drivebaseSubsystem.getPoseEstimate().getX();
    }
    @Log.Number(name = "y", index = 1)
    public double y(){
        return robot.drivebaseSubsystem.getPoseEstimate().getY();

    }
    @Log.Number(name = "rotation", index = 2)
    public double rotation(){
        return Math.toDegrees(robot.drivebaseSubsystem.getExternalHeading());

    }
    @Log.Number(name = "shooterspeed", index = 3)
    public double shooter(){
        return robot.shooterSubsystem.getVelocity();
    }
    @Log.Number(name = "flappos", index = 4)
    public double flap(){
        return robot.shooterSubsystem.getFlapPosition();
    }


}
