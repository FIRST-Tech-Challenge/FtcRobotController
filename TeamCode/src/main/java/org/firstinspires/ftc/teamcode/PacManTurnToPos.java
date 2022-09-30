package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Localizer;

public class PacManTurnToPos {
private double heading        = 0;
private double headingOffSet = 0;
private double compassHeading = 0;
private Localizer localizer   = null;
CompassSensor                compass;

    public PacManTurnToPos(HardwareMap hardwareMap, OpMode opMode, Gamepad gamepad){
        localizer = new Localizer(hardwareMap);
        localizer.handleTracking();
        compassHeading = compass.getDirection();
        headingOffSet  = localizer.heading - compassHeading;
        heading = compassHeading + headingOffSet;
    }
}
