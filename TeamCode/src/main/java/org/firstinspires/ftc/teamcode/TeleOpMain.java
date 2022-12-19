package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DriveTele;
import org.firstinspires.ftc.teamcode.commands.UpdatePose;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.OdometrySystem;
import org.firstinspires.ftc.teamcode.systems.SignalSystem;

@TeleOp(name = "Main TeleOp")
public class TeleOpMain extends CommandOpMode {
    ElapsedTime elapsedTime;
    
    DriveSystem driveSystem;
    GamepadEx driveOp;

    ElevatorSystem elevatorSystem;
    
    OdometrySystem odometrySystem;
    Pose2d start;
    
    @Override
    public void initialize() {
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        driveSystem = new DriveSystem(hardwareMap, false);
        driveOp = new GamepadEx(gamepad1);
        
        odometrySystem = new OdometrySystem(hardwareMap, start, elapsedTime);

        elevatorSystem = new ElevatorSystem(hardwareMap);
        
        schedule(
            new UpdatePose(odometrySystem),
            new WaitUntilCommand(this::isStarted),
            new DriveTele(driveSystem, elevatorSystem, driveOp)
        );
    }
}
