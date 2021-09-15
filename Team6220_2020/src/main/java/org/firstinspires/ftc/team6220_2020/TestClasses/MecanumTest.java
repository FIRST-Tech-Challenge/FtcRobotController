package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_2020.MasterTeleOp;

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

