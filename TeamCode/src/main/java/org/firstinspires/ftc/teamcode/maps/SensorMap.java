package org.firstinspires.ftc.teamcode.maps;

/**
 * Contains all sensors
 * <pre>
 *  {@code
 *  CAM("Webcam 1");
 *  }
 * </pre>
 **/
public enum SensorMap {
    // the ids of the dead wheels are from the motors
    DEAD_WHEEL_RIGHT(MotorMap.LEG_FRONT_RIGHT.getId()),
    DEAD_WHEEL_LEFT(MotorMap.LEG_FRONT_LEFT.getId()),
    DEAD_WHEEL_BACK(MotorMap.LEG_BACK_LEFT.getId());


    private final String id;

    SensorMap(String id) {
        this.id = id;
    }

    public String getId() {
        return id;
    }
}
