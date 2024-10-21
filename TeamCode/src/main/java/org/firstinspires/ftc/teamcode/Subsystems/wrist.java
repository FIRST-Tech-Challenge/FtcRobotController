package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class wrist {
    public double flexedUp= 1;
    public double rest = 0.5;
    public double flexedDown = 0;

    public Servo wrist;
    public Gamepad gamepad2;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public wrist(OpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;

        wrist = (Servo) hardwareMap.get("wristSERVO");
        wrist.setDirection(Servo.Direction.FORWARD);
    }
    public void teleOp() {
        if (gamepad2.left_stick_button) wrist.setPosition(flexedDown);
        else if (gamepad2.left_stick_button && gamepad2.right_stick_button) wrist.setPosition(rest);
        else if (gamepad2.right_stick_button) wrist.setPosition(flexedUp);
    }
}
