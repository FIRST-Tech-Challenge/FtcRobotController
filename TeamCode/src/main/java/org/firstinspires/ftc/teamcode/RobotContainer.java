package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Commands.LinearSlideMiddle;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ExampleCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.ManualDrive;
//import org.firstinspires.ftc.teamcode.Commands.ToggleClaw;
//import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawState;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.ElbowJoint;
import org.firstinspires.ftc.teamcode.Subsystems.FlappyFlappyWrist;
import org.firstinspires.ftc.teamcode.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.OctQuad;
import org.firstinspires.ftc.teamcode.Subsystems.OdometryPod;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.PivotingWrist;
import org.firstinspires.ftc.teamcode.Subsystems.ShoulderJoint;
import org.firstinspires.ftc.teamcode.Subsystems.ShoulderPosition;
import org.firstinspires.ftc.teamcode.Subsystems.SlideTargetHeight;
import org.firstinspires.ftc.teamcode.Subsystems.VirtualDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VirtualOdometry;

import java.util.ArrayList;
//import org.firstinspires.ftc.teamcode.Subsystems.LinearSlideSubsystem;


public class RobotContainer {

    // active OpMode - used so any subsystem and command and access it and its members
    public static CommandOpMode ActiveOpMode;

    // FTC dashboard and telemetries
    public static FtcDashboard DashBoard;
    public static Telemetry DBTelemetry;
    public static Telemetry RCTelemetry;

    // timer used to determine how often to run scheduler periodic
    private static ElapsedTime timer;
    private static ElapsedTime exectimer;

    // create robot GamePads
    public static GamepadEx driverOp;
    public static GamepadEx toolOp;

    // create pointers to robot subsystems
    public static DriveTrain drivesystem;
    //public static VirtualDriveTrain drivesystem;
    public static Gyro gyro;
    public static OctQuad odometryPod;
    public static Odometry odometry;
    //public static VirtualOdometry odometry;
    public static LinearSlide linearSlide;
    //public static Camera frontCamera;

    public static PivotingWrist wristRotateServo;

    /**0° is in*/
    public static FlappyFlappyWrist flappyFlappyWrist;

    /**0° is up*/
    public static ShoulderJoint shoulderJoint;

    /**0° is down*/
    public static ElbowJoint elbowJoint;

    public static Claw claw;
    //

    // Robot initialization for teleop - Run this once at start of teleop
    public static void Init_TeleOp(CommandOpMode mode) {
        // Initialize robot subsystems
        Init(mode);

        // set drivetrain default command to manual driving mode
        drivesystem.setDefaultCommand(new ManualDrive());

        // bind commands to buttons
        // bind gyro reset to back button.
        // Note: since reset is very simple command, we can just use 'InstandCommand'
        // instead of creating a full command, just to run one line of java code.
        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new InstantCommand(()-> gyro.resetYawAngle(), gyro));


        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(()->linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO)));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(()->linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW)));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(()->linearSlide.moveTo(SlideTargetHeight.SAMPLE_MEDIUM)));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(()->linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)));


        driverOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(()->wristRotateServo.RotateTo(125)));

        driverOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ArmStowHigh());

        driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(()->elbowJoint.RotateTo(0)));

        driverOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(()->shoulderJoint.RotateTo(55)));

        // Controls the claw using bumpers
        // left = close
        // right = open
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(()->claw.ControlClaw(ClawState.CLOSE)));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(()->claw.ControlClaw(ClawState.OPEN)));



        // driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new ToggleClaw());

        // example sequential command
        //driverOp.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new ExampleCommandGroup());

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
        // Initialize robot subsystems
        Init(mode);
    }

    // robot initialization - common to both auto and teleop
    private static void Init(CommandOpMode mode) {
        // save pointer to active OpMode
        ActiveOpMode = mode;

        // create and reset timer
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        exectimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();

        // set up dashboard and various telemetries
        DashBoard = FtcDashboard.getInstance();
        DBTelemetry = DashBoard.getTelemetry();
        RCTelemetry = ActiveOpMode.telemetry;

        // cancel any commands previously running by scheduler
        CommandScheduler.getInstance().cancelAll();

        // create gamepads
        driverOp = new GamepadEx(ActiveOpMode.gamepad1);
        toolOp = new GamepadEx(ActiveOpMode.gamepad2);

        // create systems
        gyro = new Gyro();
        odometryPod = new OctQuad();
        odometry = new Odometry();
        drivesystem = new DriveTrain();
        //odometry = new VirtualOdometry();
        //drivesystem = new VirtualDriveTrain();
        //frontCamera = new Camera("CamyCamy");
        linearSlide = new LinearSlide();
        flappyFlappyWrist = new FlappyFlappyWrist();
        shoulderJoint = new ShoulderJoint();
        wristRotateServo= new PivotingWrist();
        elbowJoint = new ElbowJoint();
        claw = new Claw();
        // insert other subsystems here
        // claw = new Claw();

    }


    // call this function periodically to operate scheduler
    public static void Periodic() {

        // actual interval time
        double intervaltime = timer.milliseconds();

        // execute robot periodic function 50 times per second (=50Hz)
        if (intervaltime>=20.0) {

            // reset timer
            timer.reset();

            // start execution timer
            exectimer.reset();

            // run scheduler
            CommandScheduler.getInstance().run();

            // report time interval on robot controller
            RCTelemetry.addData("interval time(ms)", intervaltime);
            RCTelemetry.addData("execute time(ms)", exectimer.milliseconds());
            RCTelemetry.update();
        }
    }

}
