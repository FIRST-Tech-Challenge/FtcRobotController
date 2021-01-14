package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Crash Test", group ="testing")
@Disabled
public class CrashTest extends AutoBase {
    int i = 0;
    public void runOpMode()
    {
        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Loop count", i);
            telemetry.update();
            i++;
            sleep(5);
        }
    }
}
