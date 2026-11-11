/*
 * Copyright (c) 2025 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This OpMode illustrates how to use DigitalChickenLabs OctoQuad Quad Encoder & Pulse Width I/F Module
 *
 * The OctoQuad has 8 input channels that can used to read either Relative Quadrature Encoders or
 * Pulse-Width Absolute Encoder inputs.  Relative Quadrature encoders are found on most FTC motors,
 * and some stand-alone position sensors like the REV Thru-Bore encoder.  Pulse-Width encoders are
 * less common. The REV Thru-Bore encoder can provide its absolute position via a variable pulse width,
 * as can several sonar rangefinders such as the MaxBotix MB1000 series.
 *
 * Note: SDK 11.0+ requires that the OctoQuad is running firmware V3.0 or greater.
 * Visit https://github.com/DigitalChickenLabs/OctoQuad/tree/master/firmware for instruction
 * on how to upgrade your OctoQuad's firmware.
 *
 * This basic sample shows how an OctoQuad can be used to read the position of three Odometry pods
 * fitted with REV Thru-Bore encoders.  For a more advanced example with additional OctoQuad
 * capabilities, see the SensorOctoQuadAdv sample.
 *
 * This OpMode assumes the OctoQuad is attached to an I2C interface named "octoquad" in the robot config.
 *
 * The code assumes the first three OctoQuad inputs are connected as follows
 * - Chan 0: for measuring forward motion on the left side of the robot.
 * - Chan 1: for measuring forward motion on the right side of the robot.
 * - Chan 2: for measuring Lateral (strafing) motion.
 *
 * The encoder values may be reset to zero by pressing the X (left most) button on Gamepad 1.
 *
 * This sample does not show how to interpret these readings, just how to obtain and display them.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.tindie.com/products/35114/
 */
@Disabled
@TeleOp(name = "OctoQuad Basic", group="OctoQuad")
public class SensorOctoQuad extends LinearOpMode {

    // Identify which encoder OctoQuad inputs are connected to each odometry pod.
    private final int ODO_LEFT  = 0; // Facing forward direction on left side of robot
    private final int ODO_RIGHT = 1; // Facing forward direction on right side or robot
    private final int ODO_PERP  = 2; // Facing perpendicular direction at the center of the robot

    // Declare the OctoQuad object;
    private OctoQuad    octoquad;

    private int         posLeft;
    private int         posRight;
    private int         posPerp;
    private int         velLeft;
    private int         velRight;
    private int         velPerp;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // Connect to OctoQuad by referring to its name in the Robot Configuration.
        octoquad = hardwareMap.get(OctoQuad.class, "octoquad");

        // Read the Firmware Revision number from the OctoQuad and display it as telemetry.
        telemetry.addData("OctoQuad Firmware Version ", octoquad.getFirmwareVersion());

        // Reverse the count-direction of any encoder that is not what you require.
        // e.g. if you push the robot forward and the left encoder counts down, then reverse it.
        octoquad.setSingleEncoderDirection(ODO_LEFT,  OctoQuad.EncoderDirection.REVERSE);
        octoquad.setSingleEncoderDirection(ODO_RIGHT, OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(ODO_PERP,  OctoQuad.EncoderDirection.FORWARD);

        // set the interval over which pulses are counted to determine velocity.
        octoquad.setAllVelocitySampleIntervals(50);  // 50 mSec means 20 velocity updates per second.

        // Save any changes that are made, just in case there is a sensor power glitch.
        octoquad.saveParametersToFlash();

        telemetry.addLine("\nPress START to read encoder values");
        telemetry.update();

        waitForStart();

        // Configure the telemetry for optimal display of data.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(100);

        // Set all the encoder inputs to zero.
        octoquad.resetAllPositions();

        // Loop while displaying the odometry pod positions.
        while (opModeIsActive()) {
            telemetry.addData(">", "Press X to Reset Encoders\n");

            // Check for X button to reset encoders.
            if (gamepad1.x) {
                // Reset the position of all encoders to zero.
                octoquad.resetAllPositions();
            }

            // Read all the encoder data.  Load into local members.
            readOdometryPods();

            // Display the values.
            telemetry.addData("Left  P", "%7d   V :%6d CPS ", posLeft, velLeft);
            telemetry.addData("Right P", "%7d   V :%6d CPS ", posRight, velRight);
            telemetry.addData("Perp  P", "%7d   V :%6d CPS ", posPerp, velPerp);
            telemetry.update();
        }
    }

    private void readOdometryPods() {
        // For best performance, we should only perform ONE transaction with the OctoQuad each cycle.
        //  This can be achieved in one of two ways:
        //   1) by doing a block data read once per control cycle
        //  or
        //   2) by doing individual caching reads, but only reading each encoder value ONCE per cycle.
        //
        // Since method 2 is simplest, we will use it here.
        posLeft  = octoquad.readSinglePosition_Caching(ODO_LEFT);
        posRight = octoquad.readSinglePosition_Caching(ODO_RIGHT);
        posPerp  = octoquad.readSinglePosition_Caching(ODO_PERP);
        velLeft  = octoquad.readSingleVelocity_Caching(ODO_LEFT)  * 20;  // scale up to cps
        velRight = octoquad.readSingleVelocity_Caching(ODO_RIGHT) * 20;  // scale up to cps
        velPerp  = octoquad.readSingleVelocity_Caching(ODO_PERP)  * 20;  // scale up to cps
    }
}
