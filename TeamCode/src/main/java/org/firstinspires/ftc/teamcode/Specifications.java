package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.core.RobotSpecifications;
import com.bravenatorsrobotics.drive.FourWheelDrive;

public class Specifications extends RobotSpecifications {

    public Specifications() {
        super(new String[] { "!fl", "fr", "!bl", "br" }, FourWheelDrive.class,
                1120, 1, 3.78);

        this.useVelocity = true;
        this.maxVelocity = 3120;
    }

}