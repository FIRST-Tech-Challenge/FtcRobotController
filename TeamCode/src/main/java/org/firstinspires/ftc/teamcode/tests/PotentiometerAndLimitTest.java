package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.Potentiometer;

@TeleOp(name="Potentiometer/Limit Test", group = "Tests")
public class PotentiometerAndLimitTest extends OpMode {

    private Potentiometer potentiometer;
    private LimitSwitch limitSwitch;

    @Override
    public void init() {
        potentiometer = new Potentiometer(hardwareMap, SHOULDER_POTENTIOMETER_ID, true);
        limitSwitch = new LimitSwitch(hardwareMap, ELBOW_LIMIT_SWITCH_ID);
        telemetry.addData("This value must be 3.3", potentiometer.getMaxVoltage());
        telemetry.update();
    }

    @Override
    public void loop() {
        potentiometer.update();
        limitSwitch.update();
        telemetry.addData("Current position:", potentiometer.getAngleDegrees());
        telemetry.addData("Limit Switch pushed?:", limitSwitch.isActivated());
        telemetry.update();
    }
}
