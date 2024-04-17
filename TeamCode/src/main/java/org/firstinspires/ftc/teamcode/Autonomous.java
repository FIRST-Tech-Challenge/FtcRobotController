package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous")
public class redNearAutonomous extends LinearOpMode {
    private CoyotesRobot robot = new CoyotesRobot(this);

    /**
     * Automatically runs after pressing the "Init" button on the Control Hub
     */
    @Override
    public void runOpMode() {
        robot.init();

        // wait until the player press the start button
        waitForStart();
    }
} 