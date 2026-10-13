package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RadianLimit {
    private final double LIMITPOS; // If radian is going up the limit is goes to until it reaches set limit
    private final double LIMITNEG; // If radian is going down the limit is goes to until it reaches set limit
    private double heading;
    private Localizer loc;
    public RadianLimit(double heading) {
        this.heading = heading;
        LIMITPOS = 3.16;
        LIMITNEG = 3.12;
        loc = new Localizer(hardwareMap, "gbpoc");
    }

    public void changeHeadingPos(double newHeading){
        if (newHeading < LIMTIPOS) {
            heading = newHeading;
        }
    }

    public double giveHeading() {
        return this.heading;
    }


}
