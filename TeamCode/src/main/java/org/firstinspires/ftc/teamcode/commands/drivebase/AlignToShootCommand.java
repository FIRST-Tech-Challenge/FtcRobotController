package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//TODO this command
public class AlignToShootCommand extends Command {
    public DrivebaseSubsystem drivebaseSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public AlignToShootCommand(DrivebaseSubsystem drive, ShooterSubsystem shoot){
        addRequirements(shoot);//, drive.dummySubsystem);
        drivebaseSubsystem = drive;
        shooterSubsystem = shoot;
    }

    @Override
    public void init() {
        //calculate changes needed
    }

    @Override
    public void execute() {
        shooterSubsystem.setVelocity(1);
    }

    @Override
    public boolean isFinished() {
        //check if changes are made
        return true;//return shooterSubsystem.getVelocity()>0.9;
    }
//    @Override
//    public void end(boolean cancel) {
//        drivebaseSubsystem.setDriveSignal(new DriveSignal());
//    }
}
