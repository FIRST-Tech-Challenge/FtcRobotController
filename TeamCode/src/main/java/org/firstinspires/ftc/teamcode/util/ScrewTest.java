package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "screw test", group = "gf")
public class ScrewTest extends OpMode
{
    private TowerController towerController;
    @Override
    public void init()
    {
        towerController = new TowerController(hardwareMap);
    }

    @Override
    public void loop()
    {
        towerController.handleGamepad(gamepad2, telemetry);
    }
}
