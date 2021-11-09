package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.chassis.MecanumChassis;
@Disabled
@TeleOp(name = "Mecanum OpMode", group = "Remote")
public class MecanumOpMode extends OpMode {
    MecanumChassis chassis = new MecanumChassis();
    @Override
    public void init() {
        chassis.init(hardwareMap);
    }

    @Override
    public void loop() {
        chassis.run(gamepad1);
    }
}
