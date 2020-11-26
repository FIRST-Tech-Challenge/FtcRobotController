package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Intake extends SubSystem {

    private DcMotor intake;

    public Intake(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        intake = robot.hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void handle() {
        if (robot.gamepad2.a) {
            intake.setPower(1);
        }
        if (robot.gamepad2.y) {
            intake.setPower(0);
        }
    }

    @Override
    public void stop() {

    }
}
