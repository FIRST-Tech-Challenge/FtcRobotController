package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;

//@Disabled
@Autonomous(name = "TurnDegrees")
public class TurnDegrees extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        turnToAngle(45);

        sleep(1000);

        turnToAngle(0);

        sleep(1000);

        turnToAngle(-45);

        sleep(1000);

        turnToAngle(0);

        sleep(1000);

        turnToAngle(90);

        sleep(1000);

        turnToAngle(0);

        sleep(1000);

        turnToAngle(-90);

        sleep(1000);

        turnToAngle(0);

        sleep(1000);

        turnToAngle(135);

        sleep(1000);

        turnToAngle(0);

        sleep(1000);

        turnToAngle(-135);

        sleep(1000);

        turnToAngle(0);

        sleep(1000);

        turnToAngle(180);

        sleep(1000);

        turnToAngle(0);

        sleep(1000);

        turnToAngle(-180);

        sleep(1000);

        turnToAngle(0);
    }
}
