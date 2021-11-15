package com.bravenatorsrobotics.freightfrenzy;

import com.bravenatorsrobotics.common.core.RobotSpecifications;
import com.bravenatorsrobotics.common.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Specifications extends RobotSpecifications {

    public Specifications() {
        super(MecanumDrive.GenerateMotors(
                "fl", true,
                "fr", false,
                "bl", true,
                "br", false
                ), MecanumDrive.class,
                1120, 1, 3.78, 11.9);

        this.useVelocity = true;
        this.maxVelocity = 2800;
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        this.debugModeEnabled = true;
    }

}