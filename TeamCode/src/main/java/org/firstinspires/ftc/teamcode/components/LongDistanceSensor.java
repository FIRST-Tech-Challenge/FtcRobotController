package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * The {@link com.qualcomm.robotcore.hardware.DistanceSensor} may be found on hardware sensors which measure distance
 * by one means or another.
 */
public interface LongDistanceSensor extends HardwareDevice
{
    /**
     * Returns the current distance in the indicated distance units
     * @param unit  the unit of distance in which the result should be returned
     * @return      the current distance sas measured by the sensor. If no reading is available
     *              (perhaps the sensor is out of range), then {@link #distanceOutOfRange} is
     *              returned;
     */
    double getDistance(DistanceUnit unit);

    /**
     * The value returned when a distance reading is not in fact available.
     */
    double distanceOutOfRange = DistanceUnit.infinity;
}
