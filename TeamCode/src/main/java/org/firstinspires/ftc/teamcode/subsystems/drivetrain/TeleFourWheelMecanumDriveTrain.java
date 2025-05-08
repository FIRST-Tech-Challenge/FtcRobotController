package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

/**
 * four wheel mecanum drive train for teleop that will drive the robot in periodic call
 */
public class TeleFourWheelMecanumDriveTrain extends BasicDriveTrain {

    @Override
    public void periodic() {
        super.periodic();

        driveRobotCentric(directionFlag * powerRatio * gamepad.getLeftX(), directionFlag * powerRatio * gamepad.getLeftY(), powerRatio * gamepad.getRightX() * -1);
    }

    public TeleFourWheelMecanumDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, LimeLight limeLight) {
        super(hardwareMap, gamepad, telemetry, feedback, false, limeLight);
    }

    public void setWheelsPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        fL.motor.setPower(frontLeftPower);
        fR.motor.setPower(frontRightPower);
        bL.motor.setPower(backLeftPower);
        bR.motor.setPower(backRightPower);
    }
}
