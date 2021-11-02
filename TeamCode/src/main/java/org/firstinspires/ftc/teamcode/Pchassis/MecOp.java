package org.firstinspires.ftc.teamcode.Pchassis;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class MecOp extends OpMode {
    public Hardware h;

    @Override
    public void init() {
        h.init(hardwareMap);
    }

    @Override
    public void loop() {
        h.m.driveRobotCentric(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
    }
}
