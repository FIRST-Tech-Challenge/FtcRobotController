package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.core.RobotSpecifications;
import com.bravenatorsrobotics.drive.MecanumDrive;
import com.bravenatorsrobotics.drive.TwoWheelDrive;

public class TwoWheelSpecs extends RobotSpecifications {

    public TwoWheelSpecs() {
        super(TwoWheelDrive.GenerateMotors(
                "left", true,
                "right", false
                ), TwoWheelDrive.class,
                288, 1, 3.520, 12.0);

        this.useVelocity = true;
        this.maxVelocity = 3120;
        this.debugModeEnabled = true;
    }

}
