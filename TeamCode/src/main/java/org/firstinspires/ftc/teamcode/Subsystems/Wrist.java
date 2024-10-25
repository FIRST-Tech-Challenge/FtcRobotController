package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Wrist {
    public double flexedUp= 0;
    public double rest = 0.5;
    public double flexedDown = 1;

    public Servo wrist;
    public Gamepad gamepad2;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public Wrist(OpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;

        wrist = (Servo) hardwareMap.get("wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
    }
    public void teleOp() {
        if (gamepad2.a) wrist.setPosition(flexedDown);
        if (gamepad2.x ) wrist.setPosition(rest);
        if (gamepad2.y) wrist.setPosition(flexedUp);
    }
}
