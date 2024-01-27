package org.firstinspires.ftc.teamcode.subbys;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveMotorSubsystem extends SubsystemBase {

    private Motor fL, bL, fR, bR;

    private SensorRevTOFDistance dist;

    public DriveMotorSubsystem(Motor fLe, Motor fRi, Motor bLe, Motor bRi, SensorRevTOFDistance d) {
        fL = fLe;
        fR = fRi;
        bL = bLe;
        bR = bRi;
        dist = d;
    }

    public void setMotorSpeed(double fLV, double fRV, double bLV, double bRV) {
        fL.set(fLV);
        fR.set(fRV);
        bL.set(bLV);
        bR.set(bRV);
    }

    public void stop() {
        fL.set(0);
        fR.set(0);
        bL.set(0);
        bR.set(0);
    }

    public double getDist(){
        return dist.getDistance(DistanceUnit.INCH);
    }

}