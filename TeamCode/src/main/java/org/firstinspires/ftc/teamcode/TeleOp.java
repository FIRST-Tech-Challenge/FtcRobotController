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

        telemetry.update(); // call-back to android console
    }

    /**
     * Controls wheel movement of the robot
     */
    public void driveWheels() {
    }

    /**
     * Controls arm movement of the robot
     */
    public void moveArm() {
        grabber();
    }

    /**
     * Controls the robot's grabber
     */
    public void moveGrabber() {
    }
}