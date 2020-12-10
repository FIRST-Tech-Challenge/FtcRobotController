package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Crash Test", group ="testing")
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
            sleep(5);
        }
    }
}
