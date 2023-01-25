package org.firstinspires.ftc.teamcode.teamUtil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants.configuredSystems;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryAssembly;

public class robotConfig {
    public HardwareMap hardwareMap;
    public OpMode opmode;
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

    public log log;
    public encoderRead encoderRead;

    public mecanumDriveBase mecanum;
    public swerveDriveBase swerve;

    public robotConfig(OpMode opmode, configuredSystems... config){
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
        this.telemetry = opmode.telemetry;
        configuredSystems = new configuredSystems[config.length+2];
        configuredSystems[0] = robotConstants.configuredSystems.LOGGING;
        configuredSystems[1] = robotConstants.configuredSystems.ENCODER_READ;
        System.arraycopy(config, 0, configuredSystems, 2, config.length);

        for (configuredSystems system : configuredSystems) {
            telemetry.addLine("initialising: "+system.toString());
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

            }
            telemetry.addLine("initialised: "+system.toString());
            telemetry.update();
        }

    }
    public static void closeLogs(){
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

}
