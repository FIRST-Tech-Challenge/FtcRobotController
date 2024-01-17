package org.firstinspires.ftc.teamcode.TeleOps.MainCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class    Outtake extends SubsystemBase {

    private static final double L_OPEN_POS = 0.7;
    private static final double L_CLOSED_POS = 0.4;
    private static final double R_OPEN_POS = 0.4;
    private static final double R_CLOSED_POS = 0.6;
    private ServoImplEx leftOuttake;
    private ServoImplEx rightOuttake;   
    private GamepadEx gamepad;

    public Outtake(GamepadEx gamepad, HardwareMap hardwareMap) {
        leftOuttake = hardwareMap.get(ServoImplEx.class, "OL");
        rightOuttake = hardwareMap.get(ServoImplEx.class, "OR");

        this.gamepad = gamepad;

        resetPosition();
    }

    public void open() {
        leftOuttake.setPosition(L_OPEN_POS);
        rightOuttake.setPosition(R_OPEN_POS);
    }

    public void close() {
        leftOuttake.setPosition(L_CLOSED_POS);
        rightOuttake.setPosition(R_CLOSED_POS);
    }

    public void resetPosition() {
       //Needs implementation
    }

    public void update() {
        // Add any real-time updates or controls here
        if (gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            open();
        } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            close();
        }
    }
}
