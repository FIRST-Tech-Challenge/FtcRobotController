package org.firstinspires.ftc.teamcode.opModes.team1.test;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;

import org.firstinspires.ftc.teamcode.components.LiftComponent;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

@TeleOp(name="Find lift positions", group="Test")
public class FindLiftPositions extends TeleOpModeBase {
    LiftComponent lift;
    Motor liftMotor;

    private final double SPEED_MULTIPLIER = 0.3;
    @Override
    public void setup() {
        liftMotor = HardwareMapContainer.motor2;
        lift = new LiftComponent(liftMotor, LiftComponent.LiftPosition.GROUND);
    }

    @Override
    public void every_tick() {
        double speed = SPEED_MULTIPLIER * Inputs.gamepad1.getLeftY();
        lift.motor.setRunMode(Motor.RunMode.RawPower);
        lift.motor.set(speed);
        telemetry.addData("Input speed", speed);
        telemetry.addData("Lift counts", liftMotor.getCurrentPosition());
    }
}
