package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Lift extends SubSystem {

    private DcMotor lift;

    @Override
    public void init() {
        lift = robot.hardwareMap.dcMotor.get("lift");
    }

    @Override
    public void handle() {

        lift.setPower(robot.gamepad2.left_stick_y);

    }

    @Override
    public void stop() {
        lift.setPower(0);
    }

    public Lift(Robot robot) { super(robot); };



}
