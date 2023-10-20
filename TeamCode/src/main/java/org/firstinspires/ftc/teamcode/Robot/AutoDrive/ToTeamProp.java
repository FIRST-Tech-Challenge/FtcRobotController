/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Robot.AutoDrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utility.Drivetrain;
import org.firstinspires.ftc.teamcode.Utility.VisionProcessorCustom;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Modified by
 * Matha Goram
 * 2023-10-14 0.2 with code snippets from the following reference document:
 * This example drives the test case for FTC OpenCV apps using a custom Vision Portal
 * illustrated by Alan G Smith in his book Learn Java for FTC
 * https://raw.githubusercontent.com/alan412/LearnJavaForFTC/master/LearnJavaForFTC.pdf
 *
 * This OpMode has been modified to align with the idiosyncracies of baqwas
 *
 * IMU https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
 * Mecanum drivetrain
 *  heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
 *  rotX = x * cos(-heading) - y * sin(-heading)
 *  rotY = x * sin(-heading) + y * cos(-heading)
 *  denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1)
 *  leftFrontPower  = (rotY + rotX + rx) / denominator
 *  leftBackPower   = (rotY - rotX + rx) / denominator
 *  rightFrontPower = (rotY - rotX - rx) / denominator
 *  rightBackPower  = (rotY + rotX - rx) / denominator
 *  Calculate the COUNTS_PER_INCH for the motor in use
 *  https://docs.google.com/spreadsheets/d/1PRGoHqyCUkSiiUiAUla-mElgsUdoqUssUntqvU-TYFY/edit?usp=sharing
 *  5203-2402-0014	5203 Series Yellow Jacket Planetary Gear Motor, 13.7:1, 435 rpm
 */

@Autonomous(name="CENTERSTAGE: Meet 1 Auto", group="Concept", preselectTeleOp = "CENTERSTAGE: Meet 1 TeleOp")
//@Disabled
public class ToTeamProp extends LinearOpMode {

    private static final String TAG = ToTeamProp.class.getSimpleName(); // for use in logging
    // Declare OpMode members
    //Datalog datalog = new Datalog(TAG);

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;
    String webcamName = "Webcam1";      // Name of the Webcam to be set in the config
    //private DrawRectangleProcessor drawRectangleProcessor;
    private VisionProcessorCustom visionProcessorCustom;
    private VisionPortal visionPortal;
    private VisionProcessorCustom.Selected teamPropPosition;
    /*
     * The ElapsedTime class provides a simple handy timer to measure elapsed time intervals.
     * The timer does not provide events or callbacks, as some other timers do.
     * Rather, at an application- determined juncture, one can reset() the timer.
     * Thereafter, one can query the interval of wall-clock time that has subsequently elapsed
     * by calling the time(), seconds(), or milliseconds() methods.
     * The timer has nanosecond internal accuracy.
     * The precision reported by the time() method is either seconds or milliseconds, depending on how the timer is initially constructed.
     * This class is thread-safe.
     * https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/util/ElapsedTime.html
     * https://first-tech-challenge.github.io/SkyStone/org/firstinspires/ftc/robotcore/external/tfod/TFObjectDetector.html
     */
    private ElapsedTime runtime = new ElapsedTime();
    IMU imu;    // universal interface for BNO055 or BHI260
    YawPitchRollAngles lastAngles, globalAngles;

    Drivetrain drivetrain = new Drivetrain(this, imu);//imu, hardwareMap);

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        visionProcessorCustom = new VisionProcessorCustom();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, webcamName),
                visionProcessorCustom);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
        // Orthogonal # 4, Logo FORWARD, USB UP
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        /*
         * imu.resetYaw();
         */
        lastAngles = imu.getRobotYawPitchRollAngles();
        telemetry.addLine("IMU")
                .addData("Yaw", "%.1f", lastAngles.getYaw(AngleUnit.DEGREES))
                .addData("Pitch", "%.1f", lastAngles.getPitch(AngleUnit.DEGREES))
                .addData("Roll", "%.1f", lastAngles.getRoll(AngleUnit.DEGREES));
        telemetry.update();

        SelectStartPosition();          // from gamepad 1

        // equivalent to waitforStart() for Driver to press PLAY button on Driver Hub
        while (!opModeIsActive() && !isStopRequested()) {
            teamPropPosition = visionProcessorCustom.getSelection();
        }
        visionPortal.stopStreaming();
        telemetry.addData("Position", teamPropPosition);
        telemetry.update();
        runtime.reset();    // reset the timer counter

        if (opModeIsActive()) {  // run until the Driver presses the STOP button
            drivetrain.Forward(12.0);
            switch (teamPropPosition) {
                // drive forward
                case LEFT:
                    // rotation angle -90
                    break;
                case CENTER:
                    // rotation angle 0
                    break;
                case RIGHT:
                    // rotation angle 90
                    break;
                default:
                    break;
            }
            // perform rotation by set angle
            // deliver purple pixel
            // perform rotation by reverse of set angle
            // drive backward
            // if blue alliance
            // then set rotation angle to -90
            // else set rotation angle to 90
            // perform rotation
            // drive forward

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Team Prop at", visionProcessorCustom.getSelection());
            telemetry.update();
        }
    }

    /**
     * Helper method to designate the starting position of the robot on the field
     * using gamepad buttons
     *
     * Invoked by
     *  runOpMode
     */
    public void SelectStartPosition() {

        int position = 4;
        String[] positions = {"  Blue Left   X/□", "  Blue Right  Y/Δ", "  Red Left   B/O", "  Red Right   A/X"};

        telemetry.setAutoClear(true);
        telemetry.clearAll();

        while (!isStopRequested()) {
            telemetry.addLine("Select start position using XYAB Keys on gamepad 1");
            for (String s : positions) {
                telemetry.addLine(s);
            }
            telemetry.update();
            if (gamepad1.x) {
                startPosition = START_POSITION.BLUE_LEFT;
                position = 0;
                break;
            }
            if (gamepad1.y) {
                startPosition = START_POSITION.BLUE_RIGHT;
                position = 1;
                break;
            }
            if (gamepad1.b) {
                startPosition = START_POSITION.RED_LEFT;
                position = 2;
                break;
            }
            if (gamepad1.a) {
                startPosition = START_POSITION.RED_RIGHT;
                position = 3;
                break;
            }

        }
        telemetry.addData("Start position", positions[position]);
        telemetry.update();
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        public final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch        = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll         = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField globalAngle  = new Datalogger.GenericField("GlobalAngle");
        public Datalogger.GenericField deltaAngle   = new Datalogger.GenericField("DeltaAngle");
        public Datalogger.GenericField correction   = new Datalogger.GenericField("Correction");
        public Datalogger.GenericField targetTicks  = new Datalogger.GenericField("TargetTicks");
        public Datalogger.GenericField motor0       = new Datalogger.GenericField("Motor0");
        public Datalogger.GenericField motor1       = new Datalogger.GenericField("Motor1");
        public Datalogger.GenericField motor2       = new Datalogger.GenericField("Motor2");
        public Datalogger.GenericField motor3       = new Datalogger.GenericField("Motor3");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            yaw,
                            pitch,
                            roll,
                            globalAngle,
                            deltaAngle,
                            correction,
                            targetTicks,
                            motor0,
                            motor1,
                            motor2,
                            motor3
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
