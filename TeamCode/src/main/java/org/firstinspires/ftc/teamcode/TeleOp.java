package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    private CoyotesRobot robot;

    @Override
    public void init() {
        robot = new CoyotesRobot(this);
    }

    @Override
    public void loop() {
        driveWheels();
        moveArm();
        moveGrabber();

        // Update the information from the robot
        telemetry.update();
    }
}