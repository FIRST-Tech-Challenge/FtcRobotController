package org.firstinspires.ftc.teamcode.teamUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.commandControl;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.*;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants.configuredSystems;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryAssembly;

public class robotConfig {
    public HardwareMap hardwareMap;
    public OpMode opMode;
    /**
     * note that the auto clearing setting on telemetry has been turned off, and telemetry item objects need to be used to update lines with new values. allows for some better control.
     */
    public Telemetry telemetry;
    public static configuredSystems[] configuredSystems;
    public static pose2D robotPose2D;
    public static pose2D previousRobotPose2D;
    public static trajectoryAssembly currentTrajectoryAssembly;
    public static final ElapsedTime elapsedTime = new ElapsedTime();

    public arm arm;
    public wrist wrist;
    public lift lift;
    public intake intake;
    public limitSwitch limitSwitch;

    public encoderRead encoderRead;

    public mecanumDriveBase mecanum;
    public swerveDriveBase swerve;

    public gamepadEX gamepadEX1;
    public gamepadEX gamepadEX2;

    public commandControl commandControl;

    private static robotConfig r_instance;

    private boolean built = false;
    public static robotConfig getInstance(OpMode OpMode){
        if(r_instance == null){
            r_instance = new robotConfig(OpMode);
        }
        return r_instance;
    }

    public void initSystems(configuredSystems... config){
        if (built) return;
        configuredSystems = new configuredSystems[config.length+3];
        configuredSystems[0] = robotConstants.configuredSystems.LOGGING;
        configuredSystems[1] = robotConstants.configuredSystems.ENCODER_READ;
        configuredSystems[2] = robotConstants.configuredSystems.GAMEPADS;
        System.arraycopy(config, 0, configuredSystems, 3, config.length);

        Telemetry.Item currentSubsystem = telemetry.addData("initialising", "");
        StringBuilder initialisedSystems = new StringBuilder();
        Telemetry.Item initialisedSubsystems = telemetry.addData("initialised", initialisedSystems);

        for (configuredSystems system : configuredSystems) {
            currentSubsystem.setValue(system.toString());
            telemetry.update();
            switch(system){
                case MECANUM:
                    mecanum = new mecanumDriveBase(this);
                    break;

                case BOTH_MODULES:
                    swerve = new swerveDriveBase(this, robotConstants.enabledModules.BOTH);
                    break;

                case LEFT_MODULE:
                    swerve = new swerveDriveBase(this, robotConstants.enabledModules.LEFT);
                    break;

                case RIGHT_MODULE:
                    swerve = new swerveDriveBase(this, robotConstants.enabledModules.RIGHT);
                    break;

                case LIFT:
                    lift = new lift(this);
                    break;

                case WRIST:
                    wrist = new wrist(this);
                    break;

                case INTAKE:
                    intake = new intake(this);
                    break;

                case ARM:
                    arm = new arm(this);
                    break;

                case LOGGING:
                    break;

                case ENCODER_READ:
                    encoderRead = new encoderRead(this);
                    break;

                case LIMIT_SWITCH:
                    limitSwitch = new limitSwitch(this);
                    break;

                case GAMEPADS:
                    commandControl = new commandControl();
                    gamepadEX1 = new gamepadEX(opMode.gamepad1);
                    gamepadEX2 = new gamepadEX(opMode.gamepad2);
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
    private robotConfig(OpMode OpMode){
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
                    swerveDriveBase.left.log.close();
                    swerveDriveBase.right.log.close();
                    swerveDriveBase.log.close();
                    break;
                case LEFT_MODULE:
                    swerveDriveBase.left.log.close();
                    swerveDriveBase.log.close();
                    break;
                case RIGHT_MODULE:
                    swerveDriveBase.right.log.close();
                    swerveDriveBase.log.close();
                    break;
                case LIFT:
                    break;
                case WRIST:
                    break;
                case INTAKE:
                    break;
                case ARM:
                    break;
                case LOGGING:
                    break;
                case ENCODER_READ:
                    break;
                case LIMIT_SWITCH:
                    break;
            }
        }
    }

    public void systemsEndLoopUpdate(){

        for (configuredSystems system : configuredSystems) {
            switch (system){
                case MECANUM:
                    break;
                case BOTH_MODULES:
                    break;
                case LEFT_MODULE:
                    break;
                case RIGHT_MODULE:
                    break;
                case LIFT:
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
                case LOGGING:
                    break;
                case ENCODER_READ:
                    break;
                case LIMIT_SWITCH:
                    break;
                case GAMEPADS:
                    gamepadEX1.endLoopUpdate();
                    gamepadEX2.endLoopUpdate();
                    break;
            }
        }
    }
}