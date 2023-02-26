package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Team2LiftComponent;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

/**
 * Description: Test Team 2's lift component
 * Hardware:
 *  [motor0] Lift Motor (Core Hex)
 *  [motor1] Unused
 *  [motor2] Unused
 *  [motor3] Unused
 *  [servo0] Unused
 *  [servo1] Unused
 *  [servo2] Unused
 *  [servo3] Unused
 * Controls:
 *  [Left Joystick] Pull up or down to lift arm from starting position
 */
@TeleOp(name="Test Team 2 Lift", group="Test")
// @Disabled
public class TestTeam2Lift extends TeleOpModeBase { // TODO: Test

    Team2LiftComponent lift;

    @Override
    public void setup() {
        Motor lift_motor = HardwareMapContainer.motor0;
        lift = new Team2LiftComponent(lift_motor, 0.42, (int)((288 / 3) / (Math.PI*2)), 0); // Core Hex Motor has 288 counts/revolution; counts/radian = counts/revn / (radians/revn); 3:1 gear
    }

    @Override
    public void every_tick() {
        lift.setHeight(Math.abs(Inputs.gamepad1.getLeftY()));
    }
}
