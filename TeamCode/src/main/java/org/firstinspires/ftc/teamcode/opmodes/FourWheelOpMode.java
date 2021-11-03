package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.FourWheelChassis;
@Disabled
@TeleOp(name = "Four Wheel OpMode", group = "Remote")
public class FourWheelOpMode extends OpMode {
    FourWheelChassis chassis = new FourWheelChassis();
    @Override
    public void init() {
        chassis.init(hardwareMap);
    }

    @Override
    public void loop() {
        chassis.drive(gamepad1);
    }
}
