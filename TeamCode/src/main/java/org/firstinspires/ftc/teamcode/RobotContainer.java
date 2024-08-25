package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.CommandGroups.ExampleCommandGroup;
//import org.firstinspires.ftc.teamcode.Commands.LinearSlideMiddle;
import org.firstinspires.ftc.teamcode.Commands.ManualDrive;
//import org.firstinspires.ftc.teamcode.Commands.ToggleClaw;
//import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlideSubsystem;


public class RobotContainer {

    // active OpMode - used so any subsystem and command and access it and its members
    public static CommandOpMode ActiveOpMode;

    // timer used to determine how often to run scheduler periodic
    private static ElapsedTime timer;

    // create robot GamePads
    public static GamepadEx driverOp;
    public static GamepadEx toolOp;

    // create pointers to robot subsystems
    public static DriveTrain drivesystem;
    public static Gyro gyro;
//    public static Claw claw;
//
//    public static LinearSlideSubsystem linearSlide;

    // Robot initialization for teleop - Run this once at start of teleop
    public static void Init_TeleOp(CommandOpMode mode) {
        // save pointer to active OpMode
        ActiveOpMode = mode;

        // Initialize robot subsystems
        Init();

        // set drivetrain default command to manual driving mode
        drivesystem.setDefaultCommand(new ManualDrive());

        // bind commands to buttons
        // bind gyro reset to back button.
        // Note: since reset is very simple command, we can just use 'InstandCommand'
        // instead of creating a full command, just to run one line of java code.
        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()-> gyro.resetYawAngle(), gyro));
//        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new ToggleClaw());

        // example of binding more complex command to a button. This would be in a separate command file
        // driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new ExampleCommand());

        // add other button commands here
        // Note: can trigger commands on
        // whenPressed - once when button is pressed
        // whenHeld - runs command while button held, but does not restart if command ends
        // whileHeld - runs command while button held, but will restart command if it ends
        // whenReleased - runs once when button is released
        // togglewhenPressed - turns command on and off at each button press
    }


    // Robot initialization for auto - Run this once at start of auto
    public static void Init_Auto(CommandOpMode mode) {

        // save pointer to active OpMode
        ActiveOpMode = mode;

        // Initialize robot subsystems
        Init();
    }

    // robot initialization - common to both auto and teleop
    private static void Init() {
        // create and reset timer
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();

        // cancel any commands previously running by scheduler
        CommandScheduler.getInstance().cancelAll();

        // create gamepads
        driverOp = new GamepadEx(ActiveOpMode.gamepad1);
        toolOp = new GamepadEx(ActiveOpMode.gamepad2);

        // create systems
        gyro = new Gyro();
        drivesystem = new DriveTrain();
//        claw = new Claw();
        // insert other subsystems here

    }


    // call this function periodically to operate scheduler
    public static void Periodic() {

        // execute robot periodic function 50 times per second (=50Hz)
        if (timer.milliseconds()>=20.0) {
            timer.reset();
            // run scheduler
            CommandScheduler.getInstance().run();
        }
    }

}
