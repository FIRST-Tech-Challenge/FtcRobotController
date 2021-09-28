package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Basic OpMode😀", group = "Remote")
public class BasicOpMode extends OpMode {
    private Chassis chassis = new Chassis();
    @Override
    public void init() {
        chassis.init(hardwareMap);
    }

    @Override
    public void loop() {
    }
}
