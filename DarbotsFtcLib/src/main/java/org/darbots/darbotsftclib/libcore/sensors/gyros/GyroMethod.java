package org.darbots.darbotsftclib.libcore.sensors.gyros;

import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public interface GyroMethod {
    public void initGyro();
    public void updateData();
    public void setGyroContainer(GyroContainer container);
    RobotGyro.HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation();
}
