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
    DEAD_WHEEL_RIGHT("front_right"),
    DEAD_WHEEL_LEFT("front_left"),
    DEAD_WHEEL_BACK("back_right");
    private final String id;

    SensorMap(String id) {
        this.id = id;
    }

    public String getId() {
        return id;
    }
}
