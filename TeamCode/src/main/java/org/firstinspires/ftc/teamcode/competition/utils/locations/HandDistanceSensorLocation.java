package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardDistanceSensor;

public class HandDistanceSensorLocation extends Location {

    private final StandardDistanceSensor SENSOR;

    public HandDistanceSensorLocation(HardwareMap hardware) {
        SENSOR = new StandardDistanceSensor(hardware, hardware.appContext.getString(R.string.HAND_DISTANCE_SENSOR));
    }

    @Override
    public void stop() {
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
