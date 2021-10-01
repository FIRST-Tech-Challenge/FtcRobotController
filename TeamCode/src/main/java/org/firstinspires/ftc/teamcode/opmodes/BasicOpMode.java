package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.chassis.Chassis;

public abstract class BasicOpMode extends OpMode {
    abstract Chassis getChassis();
    @Override
    public void init() {
        getChassis().init(hardwareMap);
    }
    @Override
    public void loop() {
        getChassis().drive(gamepad1);
    }
}
