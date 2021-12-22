package org.firstinspires.ftc.teamcode.main.utils.locations;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardDistanceSensor;

public class HandDistanceSensorLocation extends Location {

    private StandardDistanceSensor SENSOR;

    public HandDistanceSensorLocation(HardwareMap hardware) {
        try {
            SENSOR = new StandardDistanceSensor(hardware, hardware.appContext.getString(R.string.HAND_DISTANCE_SENSOR));
        } catch(Exception ignored) {}
    }

    /**
     * Returns the distance detected by the sensor.
     * @return The distance in millimeters
     */
    public double returnOutput() {
        if(SENSOR == null) {
            return 0;
        }
        return SENSOR.getDistance(DistanceUnit.MM);
    }

    @Override
    public void stop() {
        if(SENSOR == null) {
            return;
        }
        SENSOR.stop();
    }

    @Override
    public boolean isInputLocation() {
        return false;
    }

    @Override
    public boolean isOutputLocation() {
        return true;
    }

    @Override
    public InteractionSurface getInternalInteractionSurface() {
        return SENSOR;
    }

}
