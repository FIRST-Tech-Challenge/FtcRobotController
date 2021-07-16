package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.core.RobotSpecifications;
import com.bravenatorsrobotics.drive.FourWheelDrive;

public class Specifications extends RobotSpecifications {

    public Specifications() {
        super(new String[] { "left", "!right" }, FourWheelDrive.class);
    }

}