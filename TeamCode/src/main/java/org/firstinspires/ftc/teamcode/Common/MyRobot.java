package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyRobot extends Robot {

    private final OpMode opMode;

    public MyRobot(OpMode opMode) {
        this.opMode = opMode;
        CommandScheduler.getInstance().reset();
    }

    public Telemetry telemetry() {
        return opMode.telemetry;
    }

    public HardwareMap hardwareMap() {
        return opMode.hardwareMap;
    }

    public Gamepad gamepad1() {
        return opMode.gamepad1;
    }

    public Gamepad gamepad2() {
        return opMode.gamepad2;
    }
}
