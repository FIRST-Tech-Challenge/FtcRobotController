package org.firstinspires.ftc.teamcode.teamUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.*;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants.configuredSystems;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.TrajectoryAssembly;

import java.security.InvalidParameterException;

public class RobotConfig {
    public HardwareMap hardwareMap;
    public OpMode opMode;
    /**
     * note that the auto clearing setting on telemetry has been turned off, and telemetry item objects need to be used to update lines with new values. allows for some better control.
     */
    public Telemetry telemetry;
    public static configuredSystems[] configuredSystems = new configuredSystems[0];
    //public static ArrayList<configuredSystems> configuredSystems = new ArrayList<>();

    public static Pose2D
            robotPose2D,
            previousRobotPose2D;
    public static TrajectoryAssembly currentTrajectoryAssembly;
    public static final ElapsedTime elapsedTime = new ElapsedTime();

    private static Arm arm;
    private static Wrist wrist;
    private static Lift lift;
    private static Intake intake;
    private static LimitSwitch limitSwitch;

    private static EncoderRead encoderRead;

    private static MecanumDriveBase mecanum;
    private static SwerveDriveBase swerve;
    private static StandardTrackingWheelLocalizer odometry;

    private static GamepadEX
            gamepadEX1,
            gamepadEX2;

    private static CommandController commandController;

    private static RobotConfig r_instance;

    private static boolean built;

    public static RobotConfig getInstance(OpMode OpMode){
        if(r_instance == null){
            r_instance = new RobotConfig(OpMode);
        }
        return r_instance;
    }

    public <T> T getSubsystem(configuredSystems subsystem){
        Class<? extends T> desiredClass;
        T desiredSubsystem = null;

        switch (subsystem) {
            case MECANUM:
                desiredClass = (Class<? extends T>) MecanumDriveBase.class;
                desiredSubsystem = desiredClass.cast(mecanum);
                break;

            case BOTH_MODULES:
            case LEFT_MODULE:
            case RIGHT_MODULE:
                desiredClass = (Class<? extends T>) SwerveDriveBase.class;
                desiredSubsystem = desiredClass.cast(swerve);
                break;

            case LIFT:
                desiredClass = (Class<? extends T>) Lift.class;
                desiredSubsystem = desiredClass.cast(lift);
                break;

            case WRIST:
                desiredClass = (Class<? extends T>) Wrist.class;
                desiredSubsystem = desiredClass.cast(wrist);
                break;

            case INTAKE:
                desiredClass = (Class<? extends T>) Intake.class;
                desiredSubsystem = desiredClass.cast(intake);
                break;

            case ARM:
                desiredClass = (Class<? extends T>) Arm.class;
                desiredSubsystem = desiredClass.cast(arm);
                break;

            case ENCODER_READ:
                desiredClass = (Class<? extends T>) EncoderRead.class;
                desiredSubsystem = desiredClass.cast(encoderRead);
                break;

            case LIMIT_SWITCH:
                desiredClass = (Class<? extends T>) LimitSwitch.class;
                desiredSubsystem = desiredClass.cast(limitSwitch);
                break;

            case GAMEPADEX_1:
                desiredClass = (Class<? extends T>) GamepadEX.class;
                desiredSubsystem = desiredClass.cast(gamepadEX1);
                break;

            case GAMEPADEX_2:
                desiredClass = (Class<? extends T>) GamepadEX.class;
                desiredSubsystem = desiredClass.cast(gamepadEX2);
                break;
        }
        return desiredSubsystem;
    }

    public void initSystems(configuredSystems... config){
        if(built){
            throw new InvalidParameterException("Attempt to initialise additional systems intercepted.\nSystems initialisation already built.");
        }

        configuredSystems = new configuredSystems[config.length + 3];
        configuredSystems[0] = RobotConstants.configuredSystems.ENCODER_READ;
        configuredSystems[1] = RobotConstants.configuredSystems.GAMEPADEX_1;
        configuredSystems[2] = RobotConstants.configuredSystems.GAMEPADEX_2;

        System.arraycopy(config, 0, configuredSystems, 2, config.length);
        Telemetry.Item currentSubsystem = telemetry.addData("initialising", "");
        StringBuilder initialisedSystems = new StringBuilder();
        Telemetry.Item initialisedSubsystems = telemetry.addData("initialised", initialisedSystems);

        for (configuredSystems system : configuredSystems) {
            currentSubsystem.setValue(system.toString());
            telemetry.update();
            switch(system){
                case MECANUM:
                    mecanum = new MecanumDriveBase(this);
                    break;

                case BOTH_MODULES:
                    swerve = new SwerveDriveBase(this, RobotConstants.enabledModules.BOTH);
                    //odometry = new StandardTrackingWheelLocalizer(hardwareMap);
                    break;

                case LEFT_MODULE:
                    swerve = new SwerveDriveBase(this, RobotConstants.enabledModules.LEFT);
                    //odometry = new StandardTrackingWheelLocalizer(hardwareMap);
                    break;

                case RIGHT_MODULE:
                    swerve = new SwerveDriveBase(this, RobotConstants.enabledModules.RIGHT);
                    //odometry = new StandardTrackingWheelLocalizer(hardwareMap);
                    break;

                case LIFT:
                    lift = new Lift(this);
                    break;

                case WRIST:
                    wrist = new Wrist(this);
                    break;

                case INTAKE:
                    intake = new Intake(this);
                    break;

                case ARM:
                    arm = new Arm(this);
                    break;

                case ENCODER_READ:
                    encoderRead = new EncoderRead(this);
                    break;

                case LIMIT_SWITCH:
                    limitSwitch = new LimitSwitch(this);
                    break;

                case GAMEPADEX_1:
                    //commandController = new CommandController();
                    gamepadEX1 = new GamepadEX(opMode.gamepad1);
                    break;

                case GAMEPADEX_2:
                    gamepadEX2 = new GamepadEX(opMode.gamepad2);
                    break;
            }
            initialisedSystems.append("\n").append(system.toString());
            initialisedSubsystems.setValue(initialisedSystems);
            telemetry.update();
        }
        currentSubsystem.setValue("");
        telemetry.update();
    }
    private RobotConfig(OpMode OpMode){
        this.opMode = OpMode;
        this.hardwareMap = OpMode.hardwareMap;
        this.telemetry = OpMode.telemetry;
        this.telemetry.setAutoClear(false);
    }
    public void closeLogs(){
        for (configuredSystems system : configuredSystems) {
            switch (system){
                case MECANUM:
                    break;
                case BOTH_MODULES:
                    SwerveDriveBase.left.log.close();
                    SwerveDriveBase.right.log.close();
                    SwerveDriveBase.log.close();
                    break;
                case LEFT_MODULE:
                    SwerveDriveBase.left.log.close();
                    SwerveDriveBase.log.close();
                    break;
                case RIGHT_MODULE:
                    SwerveDriveBase.right.log.close();
                    SwerveDriveBase.log.close();
                    break;
                case LIFT:
                    break;
                case WRIST:
                    break;
                case INTAKE:
                    break;
                case ARM:
                    break;
                case ENCODER_READ:
                    break;
                case LIMIT_SWITCH:
                    break;
            }
        }
    }

    public void systemsStartLoopUpdate() {
        for (configuredSystems system : configuredSystems) {
            switch (system) {
                case MECANUM:
                    break;
                case BOTH_MODULES:
                case LEFT_MODULE:
                case RIGHT_MODULE:
                    //odometry.update();
                    break;
                case LIFT:
                    break;
                case WRIST:
                    break;
                case INTAKE:
                    break;
                case ARM:
                    break;
                case ENCODER_READ:
                    encoderRead.encoderBulkRead();
                    break;
                case LIMIT_SWITCH:
                    limitSwitch.startLoopUpdate();
                    break;
                case GAMEPADEX_1:
                    gamepadEX1.startLoopUpdate();
                    break;
                case GAMEPADEX_2:
                    gamepadEX2.startLoopUpdate();
                    break;
            }
        }
    }

    public void systemsEndLoopUpdate() {
        for (configuredSystems system : configuredSystems) {
            switch (system) {
                case MECANUM:
                    break;
                case BOTH_MODULES:
                    break;
                case LEFT_MODULE:
                    break;
                case RIGHT_MODULE:
                    break;
                case LIFT:

                    if(limitSwitch != null){
                        lift.update(opMode.gamepad2.right_stick_y, limitSwitch.limitSwitchEX.onPress(), lift.buttonAnalysis(opMode.gamepad2.y, opMode.gamepad2.x, opMode.gamepad2.b, opMode.gamepad2.a));
                    }
                    else{
                        lift.update(gamepadEX2.rightY.value(), false, lift.buttonAnalysis(gamepadEX2.y.isPressed(), gamepadEX2.x.isPressed(), gamepadEX2.b.isPressed(), gamepadEX2.a.isPressed()));
                    }
                    break;
                case WRIST:
                    wrist.update();
                    break;
                case INTAKE:
                    intake.update();
                    break;
                case ARM:
                    arm.update();
                    break;
                case ENCODER_READ:
                    break;
                case LIMIT_SWITCH:
                    limitSwitch.endLoopUpdate();
                    break;
                case GAMEPADEX_1:
                    gamepadEX1.endLoopUpdate();
                    break;
                case GAMEPADEX_2:
                    gamepadEX2.endLoopUpdate();
                    break;
            }
        }
    }

    public void lockDown(){
        built = false;

        mecanum = null;
        swerve = null;
        lift = null;
        wrist = null;
        intake = null;
        arm = null;
        encoderRead = null;
        limitSwitch = null;
        gamepadEX1 = null;
        gamepadEX2 = null;
    }
}