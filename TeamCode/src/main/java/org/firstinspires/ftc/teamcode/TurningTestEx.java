package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "Turning Test Ex", group = "Tests")
public class TurningTestEx extends CSMethods{
    public void runOpMode() {
        setup(true);
        turn(90);
    }
}
