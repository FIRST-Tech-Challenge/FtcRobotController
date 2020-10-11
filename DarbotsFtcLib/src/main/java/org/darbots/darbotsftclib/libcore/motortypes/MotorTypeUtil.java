package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class MotorTypeUtil {
    /**
     * Apply a gear ratio to the motorType
     * @param motorType The original motor type
     * @param ratioNumber The ratio number calculated by the radius of motor gear / radius of output gear shaft.
     * @return Applied Motor Type
     */
    public static MotorType applyGearRatio(MotorType motorType, double ratioNumber){
        return new MotorTypeImpl(motorType.getMotorName(),motorType.getCountsPerRev() * ratioNumber, motorType.getRevPerSec() / ratioNumber);
    }
}
