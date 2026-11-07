package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Classes.SWEEPRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Abstract class designed for creating new autonomous routes with SWEEP
 */
public abstract class SWEEPRoute extends LinearOpMode {
    public Sequence sequence;
    public SWEEPRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        defineRoute();
        if (sequence == null) throw new RuntimeException("Path is not defined. Override defineRoute() method and create a path.");
        if (robot == null) throw new RuntimeException("Robot is not defined. Override defineRoute() method and create a robot.");
        robot.initialize();
        waitForStart();
        robot.setPath(sequence);
        while (opModeIsActive()) {
            robot.autonomousUpdate();
        }
    }
    /**
     * This method should be overridden in subclasses to define the specific route for the robot.
     * The route should be defined using the PathBuilder class and assigned to the 'path' variable.
     */
    public void defineRoute() {
        robot = null;
        sequence = null;
    }
}
