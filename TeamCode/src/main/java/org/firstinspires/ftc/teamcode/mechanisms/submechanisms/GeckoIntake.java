package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

public class GeckoIntake  {
    private final DcMotor geckoLeft;
    private final DcMotor geckoRight;
    private final BaseRobot baseRobot;
    public GeckoIntake(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.geckoLeft = baseRobot.hardwareMap.get(DcMotor.class, Settings.Hardware.IDs.GECKO_LEFT);
        this.geckoRight = baseRobot.hardwareMap.get(DcMotor.class, Settings.Hardware.IDs.GECKO_RIGHT);

        // Reset encoders
        geckoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        geckoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        geckoRight.setDirection(DcMotorSimple.Direction.REVERSE); // so they both spin the same way to take in or spit out

        retract();

        // Set to RUN_TO_POSITION mode for position control
        geckoLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        geckoRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extend() {
    }

    public void retract() {
    }

    public void stop() {}
}
