package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

public class BasicRobot {
    public final BasicCamera camera;
    public final BasicDrivetrain drivetrain;
    public final LinearOpMode opMode;
    // public final BasicCamera camera; we are not using this class yet

    // constructor for use in autonomous mode
    public BasicRobot(LinearOpMode linearOpMode) {
        this.drivetrain = new BasicDrivetrain(linearOpMode);
        this.camera = new BasicCamera(linearOpMode);
        this.opMode = linearOpMode;
    }
    // constructor for use in teleop
    public void update() {
       this.drivetrain.update();
       // this.camera.update();
        opMode.telemetry.update();
    }

    public interface Target {
        boolean reached();
    }

    public void runUntil(Target target) {
        while (!opMode.isStopRequested() && !target.reached()) {
            update();
            opMode.telemetry.update();
        }
    }

    public void runUntilStop() {
        this.runUntil(() -> false);
    }

    public void runForTime(long millis) {
        long end = System.currentTimeMillis() + millis;
        runUntil(() -> System.currentTimeMillis() >= end);
    }

}
