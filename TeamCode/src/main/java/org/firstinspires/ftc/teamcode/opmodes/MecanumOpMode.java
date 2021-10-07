package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.MecanumChassis;

@TeleOp(name = "Mecanum OpMode \uD83D\uDE0A", group = "Remote")
public class MecanumOpMode extends BasicOpMode{
    @Override
    Chassis getChassis() {
        return new MecanumChassis();
    }
}
