package org.firstinspires.ftc.samplecode.strafer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.command.InstantCommand;
import com.technototes.library.control.gamepad.CommandGamepad;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;

import static com.technototes.subsystem.DrivebaseSubsystem.DriveSpeed.NORMAL;
import static com.technototes.subsystem.DrivebaseSubsystem.DriveSpeed.TURBO;

public class OperatorInterface {
    public Robot robot;
    public CommandGamepad driverGamepad;

    public OperatorInterface(CommandGamepad g1, CommandGamepad g2, Robot r) {
        if(g1==null){
            System.out.println("reee");
        }
        driverGamepad = g1;
        robot = r;
        setDriverControls();
    }

    public void setDriverControls() {
        driverGamepad.y.whenToggled(new InstantCommand(() -> robot.drivebaseSubsystem.driveSpeed = TURBO))
                .whenInverseToggled(new InstantCommand(() -> robot.drivebaseSubsystem.driveSpeed = NORMAL));
        driverGamepad.a.whenPressed(new InstantCommand(() -> {
            robot.drivebaseSubsystem.flMotor.zeroEncoder();
            robot.drivebaseSubsystem.frMotor.zeroEncoder();
            robot.drivebaseSubsystem.rlMotor.zeroEncoder();
            robot.drivebaseSubsystem.rrMotor.zeroEncoder();
        }));
    }

}
