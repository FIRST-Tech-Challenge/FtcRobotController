package org.firstinspires.ftc.teamcode.competition.utils.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class TeleOpEmergencyManager {

    private final Gamepad GAMEPAD1, GAMEPAD2;

    public TeleOpEmergencyManager(Gamepad gamepad1, Gamepad gamepad2) {
        GAMEPAD1 = gamepad1;
        GAMEPAD2 = gamepad2;
    }

    public boolean isSafe() {
        return !GAMEPAD1.back || !GAMEPAD2.back;
    }

}
