package org.firstinspires.ftc.teamcode.maps;

/**
 * Contains all motors
 * <pre>
 *  {@code
 *  ARM("arm", 288, 6000),
 *  JOINT("joint", 24, 500);
 *  }
 * </pre>
 **/
public enum MotorMap {
    ;
    private final String id;
    private final double maxRPM;
    private final int ticksPerRev;

    MotorMap(String id, int ticksPerRev, double maxRPM) {
        this.id = id;
        this.maxRPM = maxRPM;
        this.ticksPerRev = ticksPerRev;
    }

    public String getId() {
        return id;
    }

    public double getMaxRPM() {
        return maxRPM;
    }

    public int getTicksPerRev() {
        return ticksPerRev;
    }
}
