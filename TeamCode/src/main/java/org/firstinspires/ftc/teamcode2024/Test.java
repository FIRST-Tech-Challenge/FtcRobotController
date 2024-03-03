
/// REGULA NR 1 - TOMA SCRIE COD PERFECT
package org.firstinspires.ftc.teamcode2024;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="test", group="Linear Opmode")
public class Test extends GlobalScope2024
{
    public void runOpMode()
    {
        Initialise();
        waitForStart();
        MotorFD.setPower(0.2);
        MotorFS.setPower(0.2);
        MotorSS.setPower(0.2);
        MotorSD.setPower(0.2);
        sleep(3000);
        mb1.setPower(0.1);
        mb2.setPower(0.1);
        sleep(2000);
        while (opModeIsActive());

    }
}
