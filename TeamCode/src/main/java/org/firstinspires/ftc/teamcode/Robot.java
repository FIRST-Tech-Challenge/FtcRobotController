package org.firstinspires.ftc.teamcode;

import com.technototes.logger.Color;
import com.technototes.logger.Log;
import com.technototes.logger.Loggable;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OldDrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

/** Class for the subsystems on the robot
 *
 */
public class Robot implements Loggable {
    public Hardware hardware;

    //drivebase
    public DrivebaseSubsystem drivebaseSubsystem;

    public OdometrySubsystem odometrySubsystem;

    //index
    @Log.NumberBar(name = "Ring Capacity", index = -1, color = Color.DARK_GRAY, completeBarColor = Color.YELLOW, incompleteBarColor = Color.LIGHT_GRAY)
    public IndexSubsystem indexSubsystem;

    //intake
    @Log(name = "Intake", index = 0, color = Color.RED)
    public IntakeSubsystem intakeSubsystem;

    //shooter
    @Log(name = "Shooter", color = Color.GREEN)
    public ShooterSubsystem shooterSubsystem;

    //wobble
    @Log(name = "Wobble", index = 2, color = Color.RED)
    public WobbleSubsystem wobbleSubsystem;

    public Robot(){
        hardware = new Hardware();

        odometrySubsystem = new OdometrySubsystem(hardware.leftOdometryEncoder, hardware.rightOdometryEncoder, hardware.frontOdometryEncoder);

        drivebaseSubsystem = new DrivebaseSubsystem(hardware.flDriveMotor, hardware.frDriveMotor, hardware.rlDriveMotor, hardware.rrDriveMotor, hardware.imu, odometrySubsystem);

        indexSubsystem = new IndexSubsystem(hardware.indexPivotServo, hardware.indexArmServo);

        intakeSubsystem = new IntakeSubsystem(hardware.intakeMotorGroup);

        shooterSubsystem = new ShooterSubsystem(hardware.shooterMotorGroup, hardware.shooterFlapServo);

        wobbleSubsystem =  new WobbleSubsystem(hardware.wobbleArmServo, hardware.wobbleClawServo);

    }
}
