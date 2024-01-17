package org.firstinspires.ftc.teamcode.TeleOps.Drivebases.Swerve.DiffySwerve;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ModuleRight extends SubsystemBase {

    private DcMotorEx topModule;
    private DcMotorEx bottomModule;
    private double multiplier = 0.98;
    private ElapsedTime timer;
    private Telemetry telemetry;

    public ModuleRight(HardwareMap hardwareMap, Telemetry telemetry) {
        topModule = hardwareMap.get(DcMotorEx.class, "fr");
        bottomModule = hardwareMap.get(DcMotorEx.class, "br");
        timer = new ElapsedTime();
        this.telemetry = telemetry;

        topModule.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomModule.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void run(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        // Adjust the multiplier based on the x-axis value
        double adjustedMultiplier = multiplier * (1 - Math.abs(x));

        double topModulePower = (y * adjustedMultiplier) + rx;
        double bottomModulePower = (-y * adjustedMultiplier) + rx;

        double magnitude = Math.max(1, Math.max(topModulePower, bottomModulePower));

        topModule.setPower(topModulePower / magnitude);
        bottomModule.setPower(bottomModulePower / magnitude);

        telemetry.addData("Right Module Power", topModulePower / magnitude);
        telemetry.addData("Left Module Power", bottomModulePower / magnitude);
        telemetry.update();
    }

    public SwerveModuleState getState() {
        // Implement the logic to get the current state of the module
        // This could include reading encoder values, gyro, etc.
        // Return a SwerveModuleState object representing the module's current state

        // Replace with actual logic to obtain the current state
        double currentSpeed = 0.0;  // Replace with actual speed
        Rotation2d currentRotation = new Rotation2d(0.0);

        return new SwerveModuleState(currentSpeed, currentRotation);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Implement the logic to set the module's motors based on the desired state

        // Replace with actual logic to set motors
        topModule.setPower(desiredState.speedMetersPerSecond);
        bottomModule.setPower(desiredState.speedMetersPerSecond);
    }

    public void stop() {
        // Implement the logic to stop the module's motors

        // Replace with actual logic to stop motors
        topModule.setPower(0);
        bottomModule.setPower(0);
    }
}