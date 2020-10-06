package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MecanumTest", group = "TeleOp")
public class MecanumTest extends MasterTeleOp
{

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();

        telemetry.addData("Init", "Done");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            driveMecanumWithJoysticks();
        }
    }
}

