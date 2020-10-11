package org.darbots.darbotsftclib.libcore.templates.other_sensors;

public interface RobotGyro {
    public enum HeadingRotationPositiveOrientation{
        CounterClockwise,
        Clockwise
    }
    float getHeading();
    HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation();

}
