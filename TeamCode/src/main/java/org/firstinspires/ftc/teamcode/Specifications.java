package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.core.RobotSpecifications;
import com.bravenatorsrobotics.drive.FourWheelDrive;

public class Specifications extends RobotSpecifications {

    public Specifications() {
        super(FourWheelDrive.GenerateMotors(
                "fl", true,
                "fr", false,
                "bl", true,
                "br", false
                ), FourWheelDrive.class,
                1120, 1, 3.78, 11);

        for (String robotMotor : robotMotors) {
            System.out.println(robotMotor);
        }

        this.useVelocity = true;
        this.maxVelocity = 3120;
    }

}