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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This OpMode demonstrates how to use the "absolute localizer" feature of the
 *         Digital Chicken OctoQuad FTC Edition MK2.
 *
 * Note:  The MK2 OctoQuad has an Inertial Measurement Unit (IMU) that the MK1 does not, so this
 *        code will ONLY work with the MK2 version.
 *
 * It is STRONGLY recommended to read the Quick Start guide for the localizer feature, located here:
 * https://github.com/DigitalChickenLabs/OctoQuad/blob/master/documentation/
 *
 * This OpMode assumes that the OctoQuad is attached to an I2C interface named "octoquad" in the
 * robot configuration.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.tindie.com/products/37799/
 */

@Disabled
@TeleOp(name="OctoQuad Localizer", group="OctoQuad")
public class SensorOctoQuadLocalization extends LinearOpMode
{
    // #####################################################################################
    //
    // YOU MUST ADJUST THESE CONSTANTS FOR YOUR ROBOT!
    // SEE THE QUICKSTART GUIDE:
    //    https://github.com/DigitalChickenLabs/OctoQuad/blob/master/documentation/
    //
    // AND THE TUNING OPMODES:
    //     https://github.com/DigitalChickenLabs/OctoQuad/blob/master/code_examples/FTC
    //
    // #####################################################################################
    static final int DEADWHEEL_PORT_X = 0;  // encoder port on OctoQuad
    static final int DEADWHEEL_PORT_Y = 1;  // encoder port on OctoQuad
    static final OctoQuad.EncoderDirection DEADWHEEL_X_DIR = OctoQuad.EncoderDirection.FORWARD;
    static final OctoQuad.EncoderDirection DEADWHEEL_Y_DIR = OctoQuad.EncoderDirection.REVERSE;
    static final float X_TICKS_PER_MM  =  12.34f;  // eg.  19.89f for a goBILDA 4-Bar Odometry Pod
    static final float Y_TICKS_PER_MM  =  12.34f;  // eg.  19.89f for a goBILDA 4-Bar Odometry Pod
    static final float TCP_OFFSET_X_MM =  123.4f;  // eg. 147.0f from QuickStart Guide diagram
    static final float TCP_OFFSET_Y_MM =  123.4f;  // eg.-158.0f from QuickStart Guide diagram
    static final float IMU_SCALAR      =    1.0f;  // Rotational scale factor.
    // #####################################################################################

    // Conversion factor for radians --> degrees
    static final double RAD2DEG = 180/Math.PI;

    // For tracking the number of CRC mismatches
    int badPackets = 0;
    int totalPackets = 0;
    boolean warn = false;

    // Data structure which will store the localizer data read from the OctoQuad
    OctoQuad.LocalizerDataBlock localizer = new OctoQuad.LocalizerDataBlock();

    /*
     * Main OpMode function
     */
    public void runOpMode()
    {
        // Begin by retrieving a handle to the device from the hardware map.
        OctoQuad oq = hardwareMap.get(OctoQuad.class, "octoquad");

        // Increase the telemetry update frequency to make the data display a bit less laggy
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Configure a range of parameters for the absolute localizer
        // --> Read the quick start guide for an explanation of these!!
        // IMPORTANT: these parameter changes will not take effect until the localizer is reset!
        oq.setSingleEncoderDirection(DEADWHEEL_PORT_X, DEADWHEEL_X_DIR);
        oq.setSingleEncoderDirection(DEADWHEEL_PORT_Y, DEADWHEEL_Y_DIR);
        oq.setLocalizerPortX(DEADWHEEL_PORT_X);
        oq.setLocalizerPortY(DEADWHEEL_PORT_Y);
        oq.setLocalizerCountsPerMM_X(X_TICKS_PER_MM);
        oq.setLocalizerCountsPerMM_Y(Y_TICKS_PER_MM);
        oq.setLocalizerTcpOffsetMM_X(TCP_OFFSET_X_MM);
        oq.setLocalizerTcpOffsetMM_Y(TCP_OFFSET_Y_MM);
        oq.setLocalizerImuHeadingScalar(IMU_SCALAR);
        oq.setLocalizerVelocityIntervalMS(25);
        oq.setI2cRecoveryMode(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        // Resetting the localizer will apply the parameters configured above.
        // This function will NOT block until calibration of the IMU is complete -
        // for that you need to look at the status returned by getLocalizerStatus()
        oq.resetLocalizerAndCalibrateIMU();

        /*
         * Init Loop
         */
        while (opModeInInit())
        {
            telemetry.addData("OQ Firmware Version", oq.getFirmwareVersionString());
            telemetry.addData("Localizer Status", oq.getLocalizerStatus());
            telemetry.addData("Heading Axis Detection", oq.getLocalizerHeadingAxisChoice());
            telemetry.addLine(" ");

            warnIfNotTuned();

            telemetry.addLine("Press Play to start navigating");
            telemetry.update();

            sleep(100);
        }

        /*
         * Don't proceed to the main loop until calibration is complete
         */
        while (!isStopRequested() && (oq.getLocalizerStatus() != OctoQuad.LocalizerStatus.RUNNING))
        {
            telemetry.addLine("Waiting for IMU Calibration to complete\n");
            telemetry.addData("Localizer Status", oq.getLocalizerStatus());
            telemetry.update();
            sleep(100);
        }

        // Establish a starting position for the robot. This will be 0,0,0 by default so this line
        // is rather redundant, but you could change it to be anything you want
        oq.setLocalizerPose(0, 0, 0);

        // Not required for localizer, but left in here since raw counts are displayed
        // on telemetry for debugging
        oq.resetAllPositions();

        /*
         * MAIN LOOP
         */
        while (opModeIsActive())
        {
            // Use the Gamepad A/Cross button to teleport to a new location and heading
            if (gamepad1.crossWasPressed())
            {
                oq.setLocalizerPose(200, 200, (float)(Math.PI/2.0f));
            }

            // Read updated data from the OctoQuad into the 'localizer' data structure
            oq.readLocalizerData(localizer);

            // #################################################################################
            // IMPORTANT! Check whether the received CRC for the data is correct. An incorrect
            //            CRC indicates that the data was corrupted in transit and cannot be
            //            trusted. This can be caused by internal or external ESD.
            //
            //            If the CRC is NOT reported as OK, you should throw away the data and
            //            try to read again.
            //
            //  NOTE:  Raw odometry pod counts are displayed to verify correct direction of rotation
            //  When the robot is pushed FWD,  the X pod counts must increase in value
            //  When the robot is pushed LEFT, the Y pod counts must increase in value
            //    Use the setSingleEncoderDirection() method to make any reversals.
            // #################################################################################
            if (localizer.crcOk)
            {
                warnIfNotTuned();

                // Display Robot position data
                telemetry.addData("Localizer Status", localizer.localizerStatus);
                telemetry.addData("Heading deg", "%.2f", localizer.heading_rad * RAD2DEG);
                telemetry.addData("Rotation dps", "%.2f", localizer.velHeading_radS * RAD2DEG);
                telemetry.addData("X Pos mm", localizer.posX_mm);
                telemetry.addData("Y Pos mm", localizer.posY_mm);
                telemetry.addData("X Vel mm/s", localizer.velX_mmS);
                telemetry.addData("Y Vel mm/s", localizer.velY_mmS);
                telemetry.addLine("\nPress Gamepad A (Cross) to teleport");

                // #############################################################################
                // IMPORTANT!!
                //
                // These two encoder status lines will slow the loop down,
                // so they can be removed once the encoder direction has been verified.
                // #############################################################################
                telemetry.addData("\nRaw X Pod Counts", oq.readSinglePosition_Caching(DEADWHEEL_PORT_X));
                telemetry.addData("Raw Y Pod Counts", oq.readSinglePosition_Caching(DEADWHEEL_PORT_Y));
            }
            else
            {
                badPackets++;
                telemetry.addLine("Data CRC not valid");
            }
            totalPackets++;

            // Print some statistics about CRC validation
            telemetry.addLine(String.format("%d CRC error(s) in %d packets", badPackets, totalPackets));

            // Send updated telemetry to the Driver Station
            telemetry.update();
        }
    }

    long lastWarnFlashTimeMs = 0;
    boolean warnFlash = false;

    void warnIfNotTuned()
    {
        String warnString = "";
        if (IMU_SCALAR == 1.0f)
        {
            warnString += "WARNING: IMU_SCALAR not tuned.<br>";
            warn = true;
        }
        if (X_TICKS_PER_MM == 12.34f  ||  Y_TICKS_PER_MM == 12.34f)
        {
            warnString += "WARNING: TICKS_PER_MM not tuned.<br>";
            warn = true;
        }
        if (TCP_OFFSET_X_MM == 123.4f || TCP_OFFSET_Y_MM == 123.4f)
        {
            warnString += "WARNING: TCP_OFFSET not tuned.<br>";
            warn = true;
        }
        if (warn)
        {
            warnString +="<BR>&nbsp;-&nbsp;Read the code COMMENTS for tuning help.<BR>";

            if (System.currentTimeMillis() - lastWarnFlashTimeMs > 1000)
            {
                lastWarnFlashTimeMs = System.currentTimeMillis();
                warnFlash = !warnFlash;
            }

            telemetry.addLine(String.format("<b><font color='%s' >%s</font></b>",
                                            warnFlash ? "red" : "white", warnString));
        }
    }
}
