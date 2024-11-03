package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

/**
 * four wheel mecanum drive train for teleop that will drive the robot in periodic call
 */
public class TeleFourWheelMecanumDriveTrain extends FourWheelMecanumDrive {


    @Override
    public void periodic() {
        super.periodic();

        drive.driveRobotCentric(
                directionFlag * powerRatio * gamepad.getLeftX(),
                directionFlag * powerRatio * gamepad.getLeftY(),
                powerRatio * gamepad.getRightX() * -1
        );
    }

    public TeleFourWheelMecanumDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        super(hardwareMap, gamepad, telemetry, feedback);
    }
}
