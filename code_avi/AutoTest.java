package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CharlieTestAuto", group = "Autos")
public class AutoTest extends LinearOpMode {

    JacksonDriveTrain dt;

    @Override
    public void runOpMode() {
        dt = new JacksonDriveTrain(this);

        dt.driveAtHeading(0, 100, 0, .2);

    }
}
