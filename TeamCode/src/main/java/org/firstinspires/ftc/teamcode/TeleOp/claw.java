package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class claw {
    public Servo claw;
    public final Gamepad gamepad2;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public claw(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;

        claw = (Servo) hardwareMap.get("clawSERVO");
        claw.setDirection(Servo.Direction.FORWARD);

    }
    public void teleOp(){
        if (gamepad2.b) claw.setPosition(1);
    }
}
