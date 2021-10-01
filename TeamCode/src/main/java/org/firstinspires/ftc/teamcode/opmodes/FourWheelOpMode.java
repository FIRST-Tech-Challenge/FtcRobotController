package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.FourWheelChassis;

@TeleOp(name = "Four Wheel OpMode", group = "Remote")
public class FourWheelOpMode extends BasicOpMode{
    @Override
    Chassis getChassis() {
        return new FourWheelChassis();
    }
}
