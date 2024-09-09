package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private CoyotesRobot robot;

    /**
     * Automatically runs after pressing the "Init" button on the Control Hub
     */
    @Override
    public void runOpMode() {
        robot = new CoyotesRobot(this);

        // wait until the player press the start button
        waitForStart();
    }
} 