package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class claw {
    public Servo claw;
    public final Gamepad gamepad2;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public double OPEN = 0;
    public double CLOSE = 1;

    public claw(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;

        claw = (Servo) hardwareMap.get("clawSERVO");
        claw.setDirection(Servo.Direction.FORWARD);

    }
    public void teleOp(){
        if (gamepad2.b) claw.setPosition(CLOSE);
        else if (gamepad2.a) claw.setPosition(OPEN);
    }
}
