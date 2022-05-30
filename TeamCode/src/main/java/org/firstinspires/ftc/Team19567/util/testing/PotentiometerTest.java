
package org.firstinspires.ftc.Team19567.util.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Opmode to read really any analog input's data, but intended for use for the potentiometer and force sensor.
 * Genuinely useful!
 */

@TeleOp(name="Potentiometer Test", group="Testing")
@Disabled
public class PotentiometerTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private AnalogInput potentiometer;

    @Override
    public void init() {
        //Change deviceName to whatever analog sensor needs to be tested, or change the robot's configuration
        potentiometer = hardwareMap.get(AnalogInput.class,"potentiometer");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status","Awaiting Start");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status","OpMode Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status","Running");
        telemetry.addData("Runtime", "Run Time: " + runtime.toString());
        telemetry.addData("Potentiometer Position(%.3f)",potentiometer.getVoltage());
        telemetry.addData("Potentiometer Max Voltage",potentiometer.getMaxVoltage());
    }

    @Override
    public void stop() {
        telemetry.addData("Status","Stopped");
        telemetry.update();
    }
}
