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
    ;
    private final String id;

    SensorMap(String id) {
        this.id = id;
    }

    public String getId() {
        return id;
    }
}
