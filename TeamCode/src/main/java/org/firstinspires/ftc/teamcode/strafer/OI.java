package org.firstinspires.ftc.teamcode.strafer;

import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.simple.MecanumDriveCommand;
import com.technototes.library.control.gamepad.CommandGamepad;
import com.technototes.library.structure.OIBase;
import com.technototes.library.subsystem.drivebase.DrivebaseSubsystem;

public class OI extends OIBase {

    public Robot robot;

    public OI(CommandGamepad g1, CommandGamepad g2, Robot r) {
        super(g1, g2);
        robot = r;
        setDriverControls();
    }

    public void setDriverControls() {
//        CommandScheduler.getRunInstance().schedule(new MecanumDriveCommand(
//           robot.drivebaseSubsystem, driverGamepad.leftStick, driverGamepad.rightStick).setFieldCentric(robot.hardware.imu).addRequirements(robot.drivebaseSubsystem));
        driverGamepad.y.toggleWhenActivated(() -> robot.drivebaseSubsystem.setDriveSpeed(DrivebaseSubsystem.Speed.TURBO))
                .toggleWhenDeactivated(() -> robot.drivebaseSubsystem.setDriveSpeed(DrivebaseSubsystem.Speed.NORMAL));
    }

}
