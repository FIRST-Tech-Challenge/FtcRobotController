package org.firstinspires.ftc.teamcode.teamUtil;

import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.swerveDriveBase;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

public class encoderRead {

    List<LynxModule> allHubs;
    robotConfig r;

    public encoderRead(robotConfig r){
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

        for (robotConstants.configuredSystems subsystem: robotConfig.configuredSystems) {
            switch (subsystem){
                case BOTH_MODULES:
                case RIGHT_MODULE:
                case LEFT_MODULE:
                    swerveDriveBase.readEncoder();
                    break;
                case LIFT:
                    lift.readEncoder();
                    break;
                default:
                    break;
            }
        }
    }
}
