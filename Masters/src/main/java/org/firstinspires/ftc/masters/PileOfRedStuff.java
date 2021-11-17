package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Pile of red stuff")
public class PileOfRedStuff extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {
        // Here Wayne will do stuff eventualy
        waitForStart();
        robot.backwards(0.5, 2);
        // Read the bar code with open CV

        // right side
            robot.strafeLeft(0.5,2);
            //deposit shipping element.
            robot.turnToHeading(0.3,90,3);
            robot.forward(0.3,6);
    }
}
