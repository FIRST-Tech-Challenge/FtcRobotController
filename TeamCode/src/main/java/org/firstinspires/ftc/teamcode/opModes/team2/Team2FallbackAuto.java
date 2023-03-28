package org.firstinspires.ftc.teamcode.opModes.team2;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

public class Team2FallbackAuto extends AutonomousLinearModeBase {
    private final int NUM_ITERATIONS = 25;

    @Override
    public void run() {

        waitForStart();
        // Always travel into zone 2 by going forwards for a certain length of time
        int i = 0;

        HardwareMapContainer.motor0.set(0.5);
        HardwareMapContainer.motor1.set(0.5);

        while(opModeIsActive() && i < NUM_ITERATIONS) {
            telemetry.addData("i", i);
            telemetry.update();
            sleep(20);
            i++;
        }

        HardwareMapContainer.motor0.set(0);
        HardwareMapContainer.motor1.set(0);
    }
}
