/*
 * Copyright (c) 2024 DigitalChickenLabs
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/*
 * This OpMode illustrates how to use advanced features of the DigitalChickenLabs OctoQuad Quadrature
 *  Encoder & Pulse Width Interface Module.  The OctoQuad has 8 input channels that can used to read
 *  either Quadrature Encoder signals (like with most FTC motors) or Pulse-Width style Absolute Encoder
 *  inputs (eg: REV Through Bore encoder pulse width output).
 *
 * This OpMode illustrates several of the more advanced features of an OctoQuad, including Pulse Width
 * measurement and velocity measurement. For a more basic OpMode just showing how to read encoder
 * inputs, see the SensorOctoQuad sample.
 *
 * This OpMode assumes the OctoQuad is attached to an I2C interface named "octoquad" in the robot config.
 *
 * One system that requires a lot of encoder inputs is a Swerve Drive system.
 * Such a drive requires two encoders per drive module:
 * One encoder for the Drive motor/wheel position.velocity, and one for Steer motor angle.
 * The Drive motor requires a quadrature encoder, and the Steering motor requires an Absolute encoder.
 *
 * This sample will configure an OctoQuad for a swerve drive, as follows
 *  - Configure 4 Relative Quadrature Encoder inputs and 4 Absolute Pulse-Width Encoder inputs
 *  - Configure a velocity sample interval of 25 mSec
 *  - Configure a pulse width input range of 1-1024 uSec for a REV Encoder's Absolute Pulse output.
 *
 * An OctoSwerveDrive class will be created to initialize the OctoQuad, and manage 4 swerve modules.
 * An OctoSwerveModule class will be created to configure and read a single swerve module.
 *
 * Wiring:
 *  The OctoQuad will be configured to accept Quadrature encoders on the first four channels and
 *  Absolute (pulse width) encoders on the last four channels.
 *
 *  The standard encoder cable will connect each Driver Motor encoder to the OctoQuad. (channels 0-3)
 *
 *  A modified version of the REV 6-4 pin cable (shipped with the encoder) connects the steering encoder
 *  to the OctoQuad. (channels 4-7)
 *
 *  To connect the Absolute position signal from a REV Thru-Bore encoder to the OctoQuad, the
 *  Provided 6-pin to 4-pin cable will need to be modified.
 *  At the 6-pin connector end, move the yellow wire from its initial pin-3 position to the
 *  ABS pin-5 position. This can be done easily by using a small flathead screwdriver to lift the
 *  small white tab holding the metal wire crimp in place and gently pulling the wire out.
 *  See the OctoSwerveDrive() constructor below for the correct wheel/channel assignment.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.tindie.com/products/35114/
 */
@Disabled
@TeleOp(name="OctoQuad Advanced", group="OctoQuad")
public class SensorOctoQuadAdv extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Connect to the OctoQuad by looking up its name in the hardwareMap.
        OctoQuad octoquad = hardwareMap.get(OctoQuad.class, "octoquad");

        // Create the interface for the Swerve Drive Encoders
        OctoSwerveDrive octoSwerveDrive = new OctoSwerveDrive(octoquad);

        // Display the OctoQuad firmware revision
        telemetry.addLine("OctoQuad Firmware v" + octoquad.getFirmwareVersion());
        telemetry.addLine("\nPress START to read encoder values");
        telemetry.update();

        waitForStart();

        // Configure the telemetry for optimal display of data.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        // Run stats to determine cycle times.
        MovingStatistics avgTime = new MovingStatistics(100);
        ElapsedTime elapsedTime = new ElapsedTime();

        while (opModeIsActive()) {
            telemetry.addData(">", "Press X to Reset Encoders\n");

            if(gamepad1.x) {
                octoquad.resetAllPositions();
            }

            // read all the swerve drive encoders and update positions and velocities.
            octoSwerveDrive.updateModules();
            octoSwerveDrive.show(telemetry);

            // Update cycle time stats
            avgTime.add(elapsedTime.nanoseconds());
            elapsedTime.reset();

            telemetry.addData("Loop time", "%.1f mS", avgTime.getMean()/1000000);
            telemetry.update();
        }
    }
}

// ============================  Internal (Inner) Classes  =============================

/***
 * OctoSwerveDrive class manages 4 Swerve Modules
 * - Performs general OctoQuad initialization
 * - Creates 4 module classes, one for each swerve module
 * - Updates swerve drive status by reading all data from OctoQuad and Updating each module
 * - Displays all swerve drive data as telemetry
 */
class OctoSwerveDrive {

    private final OctoQuad octoquad;
    private final List<OctoSwerveModule> allModules = new ArrayList<>();

    // members to hold encoder data for each module.
    public final OctoSwerveModule LeftFront;
    public final OctoSwerveModule RightFront;
    public final OctoSwerveModule LeftBack;
    public final OctoSwerveModule RightBack;

    // Prepare an object to hold an entire OctoQuad encoder readable register bank (pos and vel)
    private final OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();

    public OctoSwerveDrive(OctoQuad octoquad) {
        this.octoquad = octoquad;

        // Clear out all prior settings and encoder data before setting up desired configuration
        octoquad.resetEverything();

        // Assume first 4 channels are relative encoders and the next 4 are absolute encoders
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);

        // Create the four OctoSwerveModules, and add them to a 'list' for easier reference.

        // Notes:
        //  The wheel/channel order is set here.  Ensure your encoder cables match.
        //
        //  The angleOffset must be set for each module so a forward facing wheel shows a steer
        //  angle of 0 degrees.  The process for determining the correct angleOffsets is as follows:
        //  1) Set all the angleValues (below) to zero then build and deploy the code.
        //  2) Rotate all the swerve drives so the wheels are forward in the desired 0 degree position
        //  3) Run the OpMode, view the telemetry and record the "Degrees" value for each module.
        //  4) Update the code by entering the recorded Degrees value for each module as the
        //     angleOffset (last) parameter in the lines below.
        //
        // Rebuild and deploy the new code.  Verify that the telemetry now indicates 0 degrees when
        //  the wheels are facing forward.  Also verify that the correct module values change
        //  appropriately when you manually spin (drive) and rotate (steer) a wheel.

        allModules.add(LeftFront  = new OctoSwerveModule(octoquad, "LF ",0,0));// Drive=0, Steer=4
        allModules.add(RightFront = new OctoSwerveModule(octoquad, "RF ",1,0));// Drive=1, Steer=5
        allModules.add(LeftBack   = new OctoSwerveModule(octoquad, "LB ",2,0));// Drive=2, Steer=6
        allModules.add(RightBack  = new OctoSwerveModule(octoquad, "RB ",3,0));// Drive=3, Steer=7

        // now make sure the settings persist through any power glitches.
        octoquad.saveParametersToFlash();
    }

    /**
     * Updates all 4 swerve modules
     */
    public void updateModules() {
        // Read full OctoQuad data block and then pass it to each swerve module to update themselves.
        octoquad.readAllEncoderData(encoderDataBlock);
        for (OctoSwerveModule module : allModules) {
            module.updateModule(encoderDataBlock);
        }
    }

    /**
     * Generate telemetry data for all modules.
     * @param telemetry OpMode Telemetry object
     */
    public void show(Telemetry telemetry) {
        //  create general header block and then have each module add its own telemetry
        telemetry.addData("pos", "   Count     CPS  Degree    DPS");
        for (OctoSwerveModule module : allModules) {
            module.show(telemetry);
        }
    }
}

/***
 * The OctoSwerveModule class manages a single swerve module
 */
class OctoSwerveModule {

    public  double driveCounts;
    public  double driveCountsPerSec;
    public  double steerDegrees;
    public  double steerDegreesPerSec;

    private final String   name;
    private final int      channel;
    private final double   angleOffset;

    private static final int    VELOCITY_SAMPLE_INTERVAL_MS = 25;   // To provide 40 updates/Sec.
    private static final double DEGREES_PER_US = (360.0 / 1024.0);  // REV Through Bore Encoder
    private static final double VELOCITY_SAMPLES_PER_S = (1000.0 / VELOCITY_SAMPLE_INTERVAL_MS);

    // The correct drive and turn directions must be set for the Swerve Module.
    // Forward motion must generate an increasing drive count.
    // Counter Clockwise steer rotation must generate an increasing Steer Angle (degrees)
    private static final boolean INVERT_DRIVE_ENCODER = false;
    private static final boolean INVERT_STEER_ENCODER = false;

    /***
     * @param octoquad provide access to configure OctoQuad
     * @param name name used for telemetry display
     * @param quadChannel Quadrature encoder channel.  Pulse Width channel is this + 4
     * @param angleOffset Angle to subtract from absolute encoder to calibrate zero position.
     */
    public OctoSwerveModule (OctoQuad octoquad, String name, int quadChannel, double angleOffset) {
        this.name = name;
        this.channel = quadChannel;
        this.angleOffset = angleOffset;

        // Set both encoder directions.
        octoquad.setSingleEncoderDirection(channel,
             INVERT_DRIVE_ENCODER ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(channel + 4,
             INVERT_STEER_ENCODER ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);

        // Set the velocity sample interval on both encoders
        octoquad.setSingleVelocitySampleInterval(channel, VELOCITY_SAMPLE_INTERVAL_MS);
        octoquad.setSingleVelocitySampleInterval(channel + 4, VELOCITY_SAMPLE_INTERVAL_MS);

        // Setup Absolute encoder pulse range to match REV Through Bore encoder.
        octoquad.setSingleChannelPulseWidthParams (channel + 4,
                                                    new OctoQuad.ChannelPulseWidthParams(1,1024));
    }

    /***
     * Calculate the Swerve module's position and velocity values
     * @param encoderDataBlock  most recent full data block read from OctoQuad.
     */
    public void updateModule(OctoQuad.EncoderDataBlock encoderDataBlock) {
        driveCounts = encoderDataBlock.positions[channel];
        driveCountsPerSec = encoderDataBlock.velocities[channel] * VELOCITY_SAMPLES_PER_S;

        // convert uS to degrees.  Add in any possible direction flip.
        steerDegrees = AngleUnit.normalizeDegrees(
                        (encoderDataBlock.positions[channel+ 4] * DEGREES_PER_US) - angleOffset);
        // convert uS/interval to deg per sec.  Add in any possible direction flip.
        steerDegreesPerSec = encoderDataBlock.velocities[channel + 4] *
                        DEGREES_PER_US * VELOCITY_SAMPLES_PER_S;
    }

    /**
     * Display the Swerve module's state as telemetry
     * @param telemetry OpMode Telemetry object
     */
    public void show(Telemetry telemetry) {
        telemetry.addData(name, "%8.0f %7.0f %7.0f %6.0f",
                                driveCounts, driveCountsPerSec, steerDegrees, steerDegreesPerSec);
    }
}
