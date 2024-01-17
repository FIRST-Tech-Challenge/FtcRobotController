package org.firstinspires.ftc.teamcode.robotContainer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
<<<<<<< Updated upstream
=======
import com.arcrobotics.ftclib.geometry.Rotation2d;
>>>>>>> Stashed changes
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
<<<<<<< Updated upstream
//import org.firstinspires.ftc.teamcode.commands.AutoTargetingDriveCommand;
import org.firstinspires.ftc.teamcode.Autons.BlueLeftAuton;
import org.firstinspires.ftc.teamcode.Autons.RedRightAuton;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
//import org.firstinspires.ftc.teamcode.commands.ParkingCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
=======
import org.firstinspires.ftc.teamcode.commands.AutoTargetingDriveCommand;
import org.firstinspires.ftc.teamcode.Autons.BlueLeftAuton;
import org.firstinspires.ftc.teamcode.Autons.BlueRightAuton;
import org.firstinspires.ftc.teamcode.Autons.RedLeftAuton;
import org.firstinspires.ftc.teamcode.Autons.RedRightAuton;
import org.firstinspires.ftc.teamcode.commands.AlignmentScoringCommand;
//import org.firstinspires.ftc.teamcode.commands.AutoTargetingDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
//import org.firstinspires.ftc.teamcode.commands.ParkingCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeOffAprilTags;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.changeFoldableCommand;
>>>>>>> Stashed changes
import org.firstinspires.ftc.teamcode.robotContainer.triggers.LeftTriggerTrigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


<<<<<<< Updated upstream
=======
import org.firstinspires.ftc.teamcode.robotContainer.triggers.RightTriggerTrigger;
import org.firstinspires.ftc.teamcode.subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.subsystems.AprilVision2;
>>>>>>> Stashed changes
import org.firstinspires.ftc.teamcode.subsystems.CassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DoubleCassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Config
public class RobotContainer extends Robot {
    //Basic hardware components
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;
    GamepadEx m_gamePad2;

    //Subsystems
    DriveSubsystem m_driveTrain;
    LinearSlideSubsystem m_linearSlideSubsystem;
<<<<<<< Updated upstream
    VisionSubsystem m_visionSubsystem;
  //  CassetSubsystem m_cassetSubsystem;
=======
    VisionSubsystem m_aprilVision;
    //  CassetSubsystem m_cassetSubsystem;
>>>>>>> Stashed changes
    DoubleCassetSubsystem m_cassetSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    PlaneLauncherSubsystem m_planeLauncherSubsystem;
    ClimberSubsystem m_climber;

    Command m_command;
    //DistancelocalizerSubsystem m_distancelocalizerSubsystem;
//    DistanceTrigger m_distanceTrigger;
//    TimedTrigger m_timedParkingTrigger;
    LeftTriggerTrigger m_leftTriggerTrigger;
<<<<<<< Updated upstream


    public RobotContainer(Constants.OpModeType type,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamePad1,
                        Gamepad gamePad2,
                        Pose2d initialPose,
                        double allianceHeadingOffset)
    {
=======
    RightTriggerTrigger m_rightTriggerTrigger2;
    LeftTriggerTrigger m_leftTriggerTrigger2;
    public int m_p;

    public RobotContainer(Constants.OpModeType type,
                          HardwareMap hardwareMap,
                          Telemetry telemetry,
                          Gamepad gamePad1,
                          Gamepad gamePad2,
                          Pose2d initialPose,
                          double allianceHeadingOffset) {
>>>>>>> Stashed changes
        //Initialize basic hardware structures
        m_hardwareMap = hardwareMap;
        m_gamePad1 = new GamepadEx(gamePad1);
        m_gamePad2 = new GamepadEx(gamePad2);
        m_telemetry = telemetry;

        //Setup the FTC dashboard with it's enhanced telemetry
        m_telemetry = new MultipleTelemetry(m_telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize Subsystems
        m_driveTrain = new DriveSubsystem(m_hardwareMap, m_telemetry, initialPose, allianceHeadingOffset);
        m_linearSlideSubsystem = new LinearSlideSubsystem(m_hardwareMap, m_telemetry);
<<<<<<< Updated upstream
        m_visionSubsystem = new VisionSubsystem(m_hardwareMap, m_telemetry);
=======
     //   m_visionSubsystem = new VisionSubsystem(m_hardwareMap, m_telemetry);
>>>>>>> Stashed changes
        m_cassetSubsystem = new DoubleCassetSubsystem(m_hardwareMap, m_telemetry);
        m_planeLauncherSubsystem = new PlaneLauncherSubsystem(m_hardwareMap, m_telemetry, 0);
        m_intakeSubsystem = new IntakeSubsystem(m_hardwareMap, m_telemetry);
        m_leftTriggerTrigger = new LeftTriggerTrigger(m_gamePad1, 0.05);
<<<<<<< Updated upstream
        m_climber = new ClimberSubsystem(m_hardwareMap, m_telemetry);

=======
        m_rightTriggerTrigger2 = new RightTriggerTrigger(m_gamePad2, 0.05);
        m_leftTriggerTrigger2 = new LeftTriggerTrigger(m_gamePad2, 0.05);
        m_aprilVision = new VisionSubsystem(m_hardwareMap, m_telemetry);
        m_climber = new ClimberSubsystem(m_hardwareMap, m_telemetry);
      //
>>>>>>> Stashed changes
//        m_autonParser = new AutonScriptParser(m_driveTrain,
//                                            m_clawIntakeSubsystem,
//                                            m_visionSubsystem,
//                                            m_linearSlideSubsystem);

        setupOpMode(type);
    }

    public void disableVision()
    {
<<<<<<< Updated upstream
        m_visionSubsystem.disablePipeline();
    }

=======
        m_aprilVision.disablePipeline();

    }



//    public void switchToApril(){
//        m_aprilVision.switchToApril();
//
//    }

>>>>>>> Stashed changes
//    public void setCurrentTime(double time)
//    {
//        m_timedParkingTrigger.setTime(time);
//    }

<<<<<<< Updated upstream
    public Pose2d getRobotPose()
    {
=======
    public Pose2d getRobotPose() {
>>>>>>> Stashed changes
        return m_driveTrain.getPoseEstimate();
    }

//    public void setRobotPose(Pose2d inputPose)
//    {
//        m_driveTrain.setPoseEstimate(inputPose);
//    }

<<<<<<< Updated upstream
    private void setupOpMode(Constants.OpModeType type)
    {
        if(type == Constants.OpModeType.TELEOP)
        {
            m_telemetry.addData("Initialize","TeleOp");
            setupTeleOp();
        }
        else if (type == Constants.OpModeType.BLUE_RIGHT_AUTO)
        {

            m_telemetry.addData("Initialize", "BlueRight_Auton");
            setupBlueRight_Auton();
        }
        else if (type == Constants.OpModeType.BLUE_LEFT_AUTO)
        {
            m_telemetry.addData("Initialize", "BlueLeft_Auton");
            setupBlueLeft_Auton();
        }
        else if (type == Constants.OpModeType.RED_RIGHT_AUTO)
        {
            m_telemetry.addData("Initialize", "RedRight_Auton");
            m_telemetry.addData("objectPose", m_visionSubsystem.pathNum);
            setupRedRight_Auton();
        }
        else if (type == Constants.OpModeType.RED_LEFT_AUTO)
        {
            m_telemetry.addData("Initialize", "RedLeft_Auton");
=======
    private void setupOpMode(Constants.OpModeType type) {
        if (type == Constants.OpModeType.TELEOP) {
            m_telemetry.addData("Initialize", "TeleOp");
            setupTeleOp();
        } else if (type == Constants.OpModeType.BLUE_RIGHT_AUTO) {
            setupBlueRight_Auton();
            m_telemetry.addData("Initialize", "BlueRight_Auton");
            m_telemetry.addData("location", m_aprilVision.getLocation());
            m_aprilVision.setMode("BLUE");
        } else if (type == Constants.OpModeType.BLUE_LEFT_AUTO) {
            m_telemetry.addData("Initialize", "BlueLeft_Auton");
            m_aprilVision.setMode("BLUE");
            setupBlueLeft_Auton();
        } else if (type == Constants.OpModeType.RED_RIGHT_AUTO) {
            m_telemetry.addData("Initialize", "RedRight_Auton");
            m_telemetry.addData("objectPose", m_aprilVision.getLocation());
            m_aprilVision.setMode("RED");
            setupRedRight_Auton();
        } else if (type == Constants.OpModeType.RED_LEFT_AUTO) {
            m_telemetry.addData("Initialize", "RedLeft_Auton");
            m_aprilVision.setMode("RED");
>>>>>>> Stashed changes
            setupRedLeft_Auton();
        }

        m_telemetry.update();
    }

    private void setupTeleOp() {

<<<<<<< Updated upstream
//        m_leftTriggerTrigger.whileActiveContinuous(new AutoTargetingDriveCommand(m_driveTrain,
//                m_visionSubsystem,
//                () -> m_gamePad1.getLeftY(),
//                () -> -m_gamePad1.getLeftX(),
//                () -> -m_gamePad1.getRightX(),
//                false));
=======

//        m_leftTriggerTrigger.whileActiveContinuous(new AutoTargetingDriveCommand(m_driveTrain,
//                m_aprilVision,   () -> m_gamePad1.getLeftY(),
//                () -> -m_gamePad1.getLeftX(),
//                () -> -m_gamePad1.getRightX(),
//                false));
//
//        m_leftTriggerTrigger.whileActiveContinuous(new AlignmentScoringCommand(m_driveTrain,
//                m_aprilVision,
//                false,9,false));

//        m_leftTriggerTrigger.whileActiveContinuous(new RelocalizeOffAprilTags(m_driveTrain,
//                m_aprilVision,
//                m_telemetry,9));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(
//                    m_aprilVision.doCameraSwitchingCommand("front"));
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(
//                        m_aprilVision.doCameraSwitchingCommand("back"));
>>>>>>> Stashed changes


        m_driveTrain.setDefaultCommand(new DriveCommand(m_driveTrain,
                () -> m_gamePad1.getLeftY(),
                () -> -m_gamePad1.getLeftX(),
                () -> -m_gamePad1.getRightX(),
                () -> m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                true));
<<<<<<< Updated upstream



        m_gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_planeLauncherSubsystem.shoot();}));


        m_gamePad2.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(m_intakeSubsystem.intakeCommand())
                .whenReleased(new InstantCommand(()-> {m_intakeSubsystem.stop();}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new InstantCommand(() -> {m_intakeSubsystem.outake();}))
                .whenReleased(new InstantCommand(()-> {m_intakeSubsystem.stop();}));
=======
        m_gamePad2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenHeld(new changeFoldableCommand(m_intakeSubsystem,
                () -> m_gamePad2.getLeftY(),
                () -> -m_gamePad2.getLeftX()
                ).andThen(m_intakeSubsystem.setStateCommand("intake")));



        m_rightTriggerTrigger2.whileActiveContinuous(
                m_intakeSubsystem.intakeCommand(()-> m_gamePad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))
                        ).whenInactive(()-> m_intakeSubsystem.stop());

        m_gamePad2.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new InstantCommand(() -> {
                    m_intakeSubsystem.outake();
                }))
                .whenReleased(new InstantCommand(() -> {
                    m_intakeSubsystem.stop();
                }));
>>>>>>> Stashed changes


        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(m_linearSlideSubsystem.changeLevelCommand("HIGH"));
//
//
        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(m_linearSlideSubsystem.changeLevelCommand("LOW"));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)


<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
                .whenPressed(m_linearSlideSubsystem.changeLevelCommand("MEDIUM"));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(m_linearSlideSubsystem.changeLevelCommand("LOWLOW"));

<<<<<<< Updated upstream
        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.extendToZero();})
                        .alongWith(m_cassetSubsystem.intakePoseCommand()));
//
//
        m_gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.extendToTarget();}));


        m_gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(m_climber.climb());

        m_gamePad2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(m_climber.retract());



=======
        m_gamePad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(m_intakeSubsystem.stateChangeCommand());


        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.extendToZero();})
                        .alongWith(m_cassetSubsystem.intakePoseCommand()));


        m_gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    m_linearSlideSubsystem.extendToTarget();
                }));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(m_climber.climb());


        m_gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .and(m_gamePad2.getGamepadButton(GamepadKeys.Button.X))
                .whenActive(m_planeLauncherSubsystem.shootCommand()
                );

        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()-> m_linearSlideSubsystem.lowerSlide()))
                .whenReleased(new InstantCommand(()-> m_linearSlideSubsystem.resetEncoder()));



//        m_gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
//                .whenPressed(m_climber.retract());
//
//
//
>>>>>>> Stashed changes
        m_gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(m_cassetSubsystem.depositRightCommand());

        m_gamePad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(m_cassetSubsystem.depositLeftPosition());

<<<<<<< Updated upstream
        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(m_cassetSubsystem.intakePoseCommand());




        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {m_driveTrain.correctHeadingOffset();}));


=======
//        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(m_cassetSubsystem.intakePoseCommand());


        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {
                    m_driveTrain.correctHeadingOffset();
                }));
>>>>>>> Stashed changes


    }


<<<<<<< Updated upstream


    private void setupBlueRight_Auton()
    {CommandScheduler.getInstance().schedule(

    );}


    private void setupBlueLeft_Auton()
    {
        BlueLeftAuton blueLeftAuton = new BlueLeftAuton(m_driveTrain, m_cassetSubsystem, m_intakeSubsystem, m_linearSlideSubsystem, m_visionSubsystem);
        CommandScheduler.getInstance().schedule(blueLeftAuton.generate());
    }

    private void setupRedRight_Auton()
    {
        RedRightAuton redRightAuton = new RedRightAuton(m_driveTrain, m_cassetSubsystem, m_intakeSubsystem, m_linearSlideSubsystem, m_visionSubsystem);
=======
    private void setupBlueRight_Auton() {
          BlueRightAuton blueRightAuton = new BlueRightAuton(m_driveTrain, m_cassetSubsystem, m_intakeSubsystem, m_linearSlideSubsystem, m_aprilVision);
            CommandScheduler.getInstance().schedule(blueRightAuton.generate());
    }


    private void setupBlueLeft_Auton() {
        BlueLeftAuton blueLeftAuton = new BlueLeftAuton(m_driveTrain, m_cassetSubsystem, m_intakeSubsystem, m_linearSlideSubsystem, m_aprilVision, m_telemetry);
        CommandScheduler.getInstance().schedule(blueLeftAuton.generate());
    }

    private void setupRedRight_Auton() {
        RedRightAuton redRightAuton = new RedRightAuton(m_driveTrain, m_cassetSubsystem, m_intakeSubsystem, m_linearSlideSubsystem, m_aprilVision, m_telemetry);
>>>>>>> Stashed changes
        CommandScheduler.getInstance().schedule(redRightAuton.generate());
    }


<<<<<<< Updated upstream

    private void setupRedLeft_Auton()
    {CommandScheduler.getInstance().schedule();}}
=======
    private void setupRedLeft_Auton() {
        RedLeftAuton redLeftAuton = new RedLeftAuton(m_driveTrain, m_cassetSubsystem, m_intakeSubsystem, m_linearSlideSubsystem, m_aprilVision);
        CommandScheduler.getInstance().schedule(redLeftAuton.generate());}
    }
>>>>>>> Stashed changes

