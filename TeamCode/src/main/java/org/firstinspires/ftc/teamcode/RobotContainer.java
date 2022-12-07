package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.GenericHID;
import org.firstinspires.ftc.dragonswpilib.command.button.JoystickButton;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroupDroite;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroupGauche;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroupMilieu;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.LiftDown_CMD;
import org.firstinspires.ftc.teamcode.commands.LiftStopped_CMD;
import org.firstinspires.ftc.teamcode.commands.LiftUp_CMD;
import org.firstinspires.ftc.teamcode.commands.TagDetection_CMD;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagsDetection_SS;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.dragonswpilib.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.Lift_SS;

public class RobotContainer {

    private final Gamepad mGamepad1, mGamepad2;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;
    private final DriveSubsystem mDriveSubsystem;
    private final DriveCommand mDriveCommand;
    private final AprilTagsDetection_SS m_AprilTagsDetection_SS;

    private final TagDetection_CMD mTagDetection_CMD;

    private final Lift_SS m_Lift_SS;
    private final LiftUp_CMD  m_LiftUp_CMD;
    private final LiftDown_CMD  m_LiftDown_CMD;
    private final LiftStopped_CMD m_LiftStopped_CMD;
    private final double  liftUpSpeed = 0.4;
    private final double  liftSpeedDown = -0.4;



    private int mPosition;

    private final AutonomousCommandGroupGauche mAutonomousCommandGauche;
    private final AutonomousCommandGroupDroite mAutonomousCommandGroupDroite;
    private final AutonomousCommandGroupMilieu mAutonomousCommandGroupMilieu;
    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;


        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry);
        mDriveCommand = new DriveCommand(mTelemetry, mDriveSubsystem, mGamepad1);
        mAutonomousCommandGauche = new AutonomousCommandGroupGauche(mTelemetry, mDriveSubsystem);
        mAutonomousCommandGroupDroite = new AutonomousCommandGroupDroite(mTelemetry, mDriveSubsystem);
        mAutonomousCommandGroupMilieu = new AutonomousCommandGroupMilieu(mTelemetry, mDriveSubsystem);
        m_AprilTagsDetection_SS = new AprilTagsDetection_SS(mHardwareMap, mTelemetry);

        mTagDetection_CMD  = new TagDetection_CMD(mTelemetry, m_AprilTagsDetection_SS);

        m_Lift_SS = new Lift_SS(mHardwareMap, mTelemetry);
        m_LiftUp_CMD  = new LiftUp_CMD(mTelemetry,m_Lift_SS, mGamepad1,liftUpSpeed );
        m_LiftDown_CMD  = new LiftDown_CMD(mTelemetry,m_Lift_SS, mGamepad1,liftSpeedDown );
        m_LiftStopped_CMD = new LiftStopped_CMD(mTelemetry,m_Lift_SS, mGamepad1 );


        //mPosition = m_AprilTagsDetection_SS.readTags();


        configureButtonBindings();
        configureDefaultCommands();
    }

    @Override
    protected void finalize() {
    }

    private void configureButtonBindings() {
        //JoystickButton DPadUp = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kDpadUp);
       // DPadUp.onTrue(mAutonomousCommandGroupDroite);

        JoystickButton ButtonA = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kB);
        ButtonA.whileTrue(m_LiftUp_CMD);

        JoystickButton ButtonB = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kA);
        ButtonB.whileTrue(m_LiftDown_CMD);


    }

    private void configureDefaultCommands(){
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
       // m_Lift_SS.setDefaultCommand(m_LiftStopped_CMD);
    }

    public Command getAutonomousCommand() {

        if( m_AprilTagsDetection_SS.readTags() == 1 )
        {
            return mAutonomousCommandGauche;

        } else if ( m_AprilTagsDetection_SS.readTags() == 2 ) {
            return mAutonomousCommandGroupMilieu;

        }  else if ( m_AprilTagsDetection_SS.readTags() == 3 )  {

            return mAutonomousCommandGroupDroite;

        }else {

            return mAutonomousCommandGroupMilieu;
        }


       /* switch(mPosition) {
            case 1:
                mTelemetry.addData("choix Tag ModeAuto:", mPosition);
                return mAutonomousCommandGauche;
            case 2:
                mTelemetry.addData("choix Tag ModeAuto:", mPosition);
                return mAutonomousCommandGroupMilieu;
            case 3:
                mTelemetry.addData("choix Tag ModeAuto:", mPosition);
                return mAutonomousCommandGroupDroite;
            default:
                mTelemetry.addData("choix Tag ModeAuto:", mPosition);
                return mAutonomousCommandGroupMilieu;
        }*/
    }
}
