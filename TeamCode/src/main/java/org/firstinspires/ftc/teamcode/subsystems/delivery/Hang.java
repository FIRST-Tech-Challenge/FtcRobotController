package org.firstinspires.ftc.teamcode.subsystems.delivery;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class Hang extends SonicSubsystemBase {

    private Motor left, right, pivot;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public Hang(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.left  = new Motor(hardwareMap, "left");
        this.right  = new Motor(hardwareMap, "right");
        this.pivot  = new Motor(hardwareMap, "pivot");


        left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;
    }

    public void Expand() {
        left.set(1);
        right.set(-1);
    }

    public void Collapse() {
        left.set(-1);
        right.set(1);
    }

    public void Hold() {
        left.set(0);
        right.set(0);
    }

    public void PivotExpand() {
        pivot.set(1);
    }

    public void PivotCollapse() {
        pivot.set(-1);
    }

    public void PivotHold() {
        pivot.set(0);
    }
}