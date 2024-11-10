package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class IndependentCtrlWheelMecanumDriveTrain extends FourWheelMecanumDrive {
    // y -> fl, b -> fr, x -> bl, a -> br
    private double power_fl, power_fr, power_bl, power_br;
    public IndependentCtrlWheelMecanumDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        super(hardwareMap, gamepad, telemetry, feedback, false);
    }

    @Override
    public void periodic() {
        super.periodic();
        getfL().set(power_fl);
        getfR().set(power_fr);
        getbL().set(power_bl);
        getbR().set(power_br);

    }

    public void onYPressed() {
        power_fl = 0.5;
    }

    public void onAPressed() {
        power_br = 0.5;
    }

    public void onXPressed() {
        power_bl = 0.5;
    }

    public void onBPressed() {
        power_fr = 0.5;
    }
    public void allStop() {
        power_fl = 0;
        power_fr = 0;
        power_bl = 0;
        power_br = 0;
    }
}
