package org.firstinspires.ftc.teamcode.main.utils.locations;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardDistanceSensor;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

public class IntakeLiftingDistanceSensorLocation extends Location {

    private StandardDistanceSensor SENSOR;

    public IntakeLiftingDistanceSensorLocation(HardwareMap hardware) {
        try {
            SENSOR = new StandardDistanceSensor(hardware, Resources.Intake.Sensors.LiftDistanceSensor);
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
