package org.firstinspires.ftc.teamcode.teamUtil;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

public class EncoderRead {

    List<LynxModule> allHubs;
    RobotConfig r;

    public EncoderRead(RobotConfig r){
        this.r = r;
        allHubs = r.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void encoderBulkRead(){
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        for (RobotConstants.configuredSystems system : RobotConfig.configuredSystems) {
            switch (system){
                case MECANUM:
                    break;
                case BOTH_MODULES:
                case RIGHT_MODULE:
                case LEFT_MODULE:
                    SwerveDriveBase.readEncoder();
                    break;
                case LIFT:
                    Lift.readEncoder();
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
                case GAMEPADS:
                    break;
                default:
                    break;
            }
        }
    }
}
