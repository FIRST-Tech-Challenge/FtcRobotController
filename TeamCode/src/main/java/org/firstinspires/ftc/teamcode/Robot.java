package org.firstinspires.ftc.teamcode;

import com.technototes.logger.Color;
import com.technototes.logger.Log;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

/** Class for the subsystems on the robot
 *
 */
public class Robot implements Loggable {
    public Hardware hardware;

    //drivebase
    public DrivebaseSubsystem drivebaseSubsystem;

    //index
    public IndexSubsystem indexSubsystem;

    //intake
    @Log(name = "Intake", index = 0, color = Color.RED)
    public IntakeSubsystem intakeSubsystem;

    //shooter
    public ShooterSubsystem shooterSubsystem;

    //wobble
    @Log(name = "Wobble", index = 1, color = Color.RED)
    public WobbleSubsystem wobbleSubsystem;

    public Robot(){
        hardware = new Hardware();

        drivebaseSubsystem = new DrivebaseSubsystem(hardware.flDriveMotor, hardware.frDriveMotor, hardware.rlDriveMotor, hardware.rrDriveMotor, hardware.imu);

        //indexSubsystem = new IndexSubsystem(hardware.indexMotor);

        intakeSubsystem = new IntakeSubsystem(hardware.intakeMotor);

        shooterSubsystem = new ShooterSubsystem(hardware.shooterMotor1, hardware.shooterMotor2);

        wobbleSubsystem =  new WobbleSubsystem(hardware.wobbleServo1, hardware.wobbleServo2);

    }
}
