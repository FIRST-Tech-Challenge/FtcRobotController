package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

@TeleOp(name="Test Grabber Code", group="Saku Test")
@Disabled
public class SBOTTINGOTATestGrabber extends TeleOpModeBase {
    // replace with suitable amounts (in degrees)
    public final double SERVO_OPEN_ANGLE = 0D;
    public final double SERVO_CLOSE_ANGLE = 90D;

    private ServoEx grabberServo;

    private Telemetry telemetry;

    @Override
    public void setup() {
        telemetry = TelemetryContainer.getTelemetry();

        telemetry.addData("Open Angle: ", SERVO_OPEN_ANGLE);
        telemetry.addData("Close Angle: ", SERVO_CLOSE_ANGLE);

        grabberServo = new SimpleServo(HardwareMapContainer.getMap(), "grabberServo",
                SERVO_OPEN_ANGLE, SERVO_CLOSE_ANGLE);

        new GamepadButton(Inputs.gamepad1, PSButtons.CIRCLE).whenActive(() ->
            grabberServo.setPosition(SERVO_CLOSE_ANGLE)
        );

        new GamepadButton(Inputs.gamepad1, PSButtons.SQUARE).whenActive(() ->
                grabberServo.setPosition(SERVO_OPEN_ANGLE)
        );
    }

    @Override
    public void every_tick() {
        telemetry.addData("Current Servo Angle: ", grabberServo.getAngle());
    }
}
