package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveTele;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;

@TeleOp(name = "Main TeleOp")
public class TeleOpMain extends CommandOpMode {
    DriveSystem driveSystem;
    GamepadEx driveOp;
    
    @Override
    public void initialize() {
        driveSystem = new DriveSystem(hardwareMap, false);
        driveOp = new GamepadEx(gamepad1);
        
        schedule(
                new WaitUntilCommand(this::isStarted),
                new DriveTele(driveSystem, driveOp)
        );
    }
}
