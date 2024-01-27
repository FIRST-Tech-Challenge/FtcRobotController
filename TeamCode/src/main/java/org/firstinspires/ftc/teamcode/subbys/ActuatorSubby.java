package org.firstinspires.ftc.teamcode.subbys;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ActuatorSubby extends SubsystemBase {

    Motor hang;
    SensorRevTOFDistance dist;

    public ActuatorSubby(Motor h, SensorRevTOFDistance d) {
        hang = h;
        dist = d;
    }

    public void run() {
        hang.set(1);
    }

    public void reverse() {
        hang.set(-1);
    }

    public void stop() {
        hang.set(0);
    }

    public double getDist(){
        return dist.getDistance(DistanceUnit.INCH);
    }

}