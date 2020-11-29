package org.firstinspires.ftc.teamcode.virtual;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.json.JSONException;

@TeleOp(name="Virtual Hardware")
public class VirtualHardwareTest extends VirtualHardware {

    public VirtualHardwareTest() {
        this.initVuforia = false;
    }

    @Override
    public void initializeHardware() {
        super.initializeHardware();
    }

    @Override
    public void run_loop() {
        this.testMotor.setPower(gamepad1.left_stick_y);

        try {
            vhManager.updateDevices();
        } catch (JSONException e) {
                telemetry.addData("err", "can't update devices");
        }
    }
}
