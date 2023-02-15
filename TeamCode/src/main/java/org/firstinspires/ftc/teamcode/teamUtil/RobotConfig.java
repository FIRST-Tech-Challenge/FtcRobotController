package org.firstinspires.ftc.teamcode.teamUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.CommandControl;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.*;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants.configuredSystems;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.TrajectoryAssembly;

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

    public Arm arm;
    public Wrist wrist;
    public Lift lift;
    public Intake intake;
    public LimitSwitch limitSwitch;

    public EncoderRead encoderRead;

    public MecanumDriveBase mecanum;
    public SwerveDriveBase swerve;
    public StandardTrackingWheelLocalizer odometry;

    public GamepadEX
            gamepadEX1,
            gamepadEX2;

    public CommandControl commandControl;

    private static RobotConfig r_instance;

    private boolean built = false;

    public static RobotConfig getInstance(OpMode OpMode){
        if(r_instance == null){
            r_instance = new RobotConfig(OpMode);
        }
        return r_instance;
    }

    public void initSystems(configuredSystems... config){
        try{
            if(built){
                throw new RuntimeException("Attempt to initialise additional systems intercepted.\nSystems initialisation already built.");
            }
        }
        catch (RuntimeException e){
            throw new RuntimeException(e);
        }
        configuredSystems = new configuredSystems[config.length + 2];
        configuredSystems[0] = RobotConstants.configuredSystems.ENCODER_READ;
        configuredSystems[1] = RobotConstants.configuredSystems.GAMEPADS;
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
                    odometry = new StandardTrackingWheelLocalizer(hardwareMap);
                    break;

                case LEFT_MODULE:
                    swerve = new SwerveDriveBase(this, RobotConstants.enabledModules.LEFT);
                    odometry = new StandardTrackingWheelLocalizer(hardwareMap);
                    break;

                case RIGHT_MODULE:
                    swerve = new SwerveDriveBase(this, RobotConstants.enabledModules.RIGHT);
                    odometry = new StandardTrackingWheelLocalizer(hardwareMap);
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

                case GAMEPADS:
                    commandControl = new CommandControl();
                    gamepadEX1 = new GamepadEX(opMode.gamepad1);
                    gamepadEX2 = new GamepadEX(opMode.gamepad2);
                    break;
            }
            initialisedSystems.append("\n").append(system.toString());
            initialisedSubsystems.setValue(initialisedSystems);
            telemetry.update();
        }
        currentSubsystem.setValue("");
        telemetry.update();
        built = true;
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
                    odometry.update();
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
                case GAMEPADS:
                    gamepadEX1.startLoopUpdate();
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

                    lift.update(gamepadEX2.rightY.value(), false, lift.buttonAnalysis(gamepadEX2.y.isPressed(), gamepadEX2.x.isPressed(), gamepadEX2.b.isPressed(), gamepadEX2.a.isPressed()));

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
                case GAMEPADS:
                    gamepadEX1.endLoopUpdate();
                    gamepadEX2.endLoopUpdate();
                    break;
            }
        }
    }
}