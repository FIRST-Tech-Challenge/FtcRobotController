package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Position Viewer", group = "Real")
public class PositionViewer extends OpMode {
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(this);
    }

    @Override
    public void loop() {
        robot.run();
    }
}
