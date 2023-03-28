package org.firstinspires.ftc.teamcode.opModes.team2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

@Autonomous(name="USE THIS ONE - Team 2 Fallback", group="Team 2")
public class Team2FallbackAuto extends AutonomousLinearModeBase {
    private final int NUM_ITERATIONS = 50;

    @Override
    public void run() {

        HardwareMapContainer.motor0.setInverted(true);

        waitForStart();
        // Always travel into zone 2 by going forwards for a certain length of time
        int i = 0;

        HardwareMapContainer.motor0.set(1.0);
        HardwareMapContainer.motor1.set(1.0);

        while(opModeIsActive() && i < NUM_ITERATIONS) {
            telemetry.addData("i", i);
            telemetry.update();
            sleep(20);
            i++;
        }

        HardwareMapContainer.motor0.set(0);
        HardwareMapContainer.motor1.set(0);

        // Release cone
    }
}
