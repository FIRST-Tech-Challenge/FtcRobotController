package org.firstinspires.ftc.teamcode.util.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class SympleCommandOpMode extends CommandOpMode {
    /**
     * runs when the robot exist the init mode and enters to the run mode
     */
    public void sympleStart() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        sympleStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }
}
