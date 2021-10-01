package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Basic OpMode😀", group = "Remote")
public class BasicOpMode extends OpMode {
    private final MecanumChassis mecanumChassis = new MecanumChassis();
    @Override
    public void init() {
        mecanumChassis.init(hardwareMap);
    }
    @Override
    public void loop() {
        mecanumChassis.drive(gamepad1);
    }
}
