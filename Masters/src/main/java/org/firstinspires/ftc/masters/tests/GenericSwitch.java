/**
 *
 * Created by Maddie and Bria!, FTC Team 4962, The Rockettes
 * version 1.0 Aug 22, 2016
 *
 */

package org.firstinspires.ftc.masters.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Sensor: Generic Switch", group = "Sensor")
//@Disabled
public class GenericSwitch extends LinearOpMode {

    /*
     * Main loop
     */
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the hardware
         */

        DigitalChannel genericSwitch;
        genericSwitch = hardwareMap.digitalChannel.get("switch");
        genericSwitch.setMode(DigitalChannel.Mode.INPUT);

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            // is it on or off?

            boolean isItOpen = genericSwitch.getState();

            String switchState;
            if (isItOpen) {
                switchState = "Open";
            } else {
                switchState = "Closed";
            }
            telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
            telemetry.addData("state:", switchState);
            telemetry.addData("directState:", genericSwitch.getState());
            telemetry.addData("mode:", genericSwitch.getMode());
            telemetry.update();
        }
    }

}
