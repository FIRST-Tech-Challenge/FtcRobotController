package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "Real")
public class MainOpMode extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.configureTeleOpBindings();
    }

    @Override
    public void loop() {
        robot.run();
    }
}