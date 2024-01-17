package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Launcher extends SubsystemBase {

    private Servo launchServo;
    private CRServo aimServo;
    private GamepadEx gamepad;

    public Launcher(GamepadEx gamepad, HardwareMap hardwareMap) {
        launchServo = hardwareMap.get(ServoImplEx.class, "launch");
        aimServo = hardwareMap.get(CRServoImplEx.class, "aim");

        this.gamepad = gamepad;
    }

    public void adjustAngle() {
        // Needs Implementation
        double x = gamepad.getLeftX();
        double y = gamepad.getLeftY();

        double a = Math.atan2(y, x);

        aToP(a);
    }

    public void aToP(double angle) {
        double minPower = -1.0;
        double maxPower = 1.0;

        double servoPower = angle / Math.PI;
        aimServo.setPower(servoPower);
    }

    public void launch() {
        // Trigger the servo to launch the paper airplane
        launchServo.setPosition(1.0);
    }
}
