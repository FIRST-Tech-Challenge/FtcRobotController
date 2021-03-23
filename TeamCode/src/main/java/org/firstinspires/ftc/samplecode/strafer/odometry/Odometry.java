package org.firstinspires.ftc.samplecode.strafer.odometry;
public interface Odometry {

    default Position getPosition(){
        return new Position(getX(), getY(), getRotation());
    }

    double getX();

    double getY();

    double getRotation();

    Odometry update();

}
