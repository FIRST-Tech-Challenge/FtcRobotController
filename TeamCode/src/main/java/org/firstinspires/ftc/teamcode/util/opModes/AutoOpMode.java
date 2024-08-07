package org.firstinspires.ftc.teamcode.util.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class AutoOpMode extends CommandOpMode {

    /**
     * runs when the robot exist the init mode and enters to the run mode
     */
    abstract public void sympleStart();

    /**
     * runs in loop when the robot is in the init mode
     */
    public void runInit() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // runs before you enter the init loop
        this.initialize();

        // runs when in init mode
        while (this.opModeInInit() && !this.isStopRequested()) {
            this.runInit();
        }

        this.waitForStart();

        // runs after the robot exit the init loop
        this.sympleStart();


        // run mode
        while (!this.isStopRequested() && this.opModeIsActive()) {
            this.run();
        }

        this.reset();
    }
}
