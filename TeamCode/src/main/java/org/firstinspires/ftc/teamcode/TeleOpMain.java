package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ControlGrabbing;
import org.firstinspires.ftc.teamcode.commands.DriveTele;
import org.firstinspires.ftc.teamcode.commands.UpdatePose;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.OdometrySystem;
import org.firstinspires.ftc.teamcode.systems.SignalSystem;

@TeleOp(name = "Main TeleOp")
public class TeleOpMain extends CommandOpMode {
    @Override
    public void initialize() {
        DriveSystem driveSystem = new DriveSystem(hardwareMap);
        ElevatorSystem elevatorSystem = new ElevatorSystem(hardwareMap);
        
        schedule(
            new WaitUntilCommand(this::isStarted),
            new DriveTele(driveSystem, gamepad1),
            new ControlGrabbing(elevatorSystem, gamepad2)
        );

        register(elevatorSystem);
    }
}
