package com.bravenatorsrobotics.common.utils;

import com.bravenatorsrobotics.common.operation.TeleopMode;
import com.bravenatorsrobotics.freightfrenzy.Teleop;
import com.qualcomm.robotcore.util.Range;

public class PowerScale {

    private final TeleopMode<?> teleopMode;
    private final double powerScale;

    public PowerScale(TeleopMode<?> teleopMode, double powerScale) {
        this.teleopMode = teleopMode;
        this.powerScale = powerScale;
    }

    public double GetPower(double targetPowerLevel, double currentPowerLevel) {

        if(currentPowerLevel == targetPowerLevel) {
            return targetPowerLevel;
        }

        if(currentPowerLevel < targetPowerLevel) {
            currentPowerLevel += powerScale * teleopMode.GetDeltaTime();
        } else {
            currentPowerLevel -= powerScale * teleopMode.GetDeltaTime();
        }

        if((targetPowerLevel > 0 && currentPowerLevel > targetPowerLevel) ||
           (targetPowerLevel < 0 && currentPowerLevel < targetPowerLevel)) {
            return targetPowerLevel;
        }

        return currentPowerLevel;

//        boolean isTargetNegative = targetPowerLevel < 0;
//
//        if((!isTargetNegative && currentPowerLevel >= targetPowerLevel)
//          || isTargetNegative && currentPowerLevel <= targetPowerLevel) {
//            return targetPowerLevel;
//        }
//
//        return currentPowerLevel + (powerScale * teleopMode.GetDeltaTime())
//                                                        * (isTargetNegative ? -1 : 1);
    }

}
