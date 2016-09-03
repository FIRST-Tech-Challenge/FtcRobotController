/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.external.samples;

import android.hardware.Sensor;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.*;

/**
 * This class demonstrates how to manually register opmodes.
 *
 * Note: It is NOT required to manually register OpModes, but this method is provided to support teams that
 * want to have centralized control over their opmode lists.
 * This sample is ALSO handy to use to selectively register the other sample opmodes.
 * Just copy/paste this sample to the TeamCode module and then uncomment the opmodes you wish to run.
 *
 * How it works:
 * The registerMyOpmodes method will be called by the system during the automatic registration process
 * You can register any of the opmodes in your App. by making manager.register() calls inside registerMyOpmodes();
 *
 * Note:
 * Even though you are performing a manual Registration, you should set the Annotations on your classes so they
 * can be placed into the correct Driver Station OpMode list...  eg:
 *
 * @Autonomous(name="DriveAndScoreRed", group ="Red")
 * or
 * @TeleOp(name="FastDrive", group ="A-Team")
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Then uncomment and copy the manager.register() call to register as many of your OpModes as you like.
 * You can even use it to temporarily register samples directly from the robotController/external/samples folder.
 */
public class ConceptRegisterOpModes
{

  @OpModeRegistrar
  public static void registerMyOpModes(OpModeManager manager) {
    // Un-comment any line to enable that sample.
    // Or add your own lines to register your Team opmodes.

    // Basic Templates
    // manager.register("Iterative Opmode",       TemplateOpMode_Iterative.class);
    // manager.register("Linear Opmode",          TemplateOpMode_Linear.class);

    // Driving Samples
    // manager.register("Teleop POV",             PushbotTeleopPOV_Linear.class);
    // manager.register("Teleop Tank",            PushbotTeleopTank_Iterative.class);
    // manager.register("Auto Drive Gyro",        PushbotAutoDriveByGyro_Linear.class);
    // manager.register("Auto Drive Encoder",     PushbotAutoDriveByEncoder_Linear.class);
    // manager.register("Auto Drive Time",        PushbotAutoDriveByTime_Linear.class);
    // manager.register("Auto Drive Line",        PushbotAutoDriveToLine_Linear.class);
    // manager.register("K9 Telop",               K9botTeleopTank_Linear.class);

    // Sensor Samples
    // manager.register("AdaFruit Color",         SensorAdafruitRGB.class);
    // manager.register("HT Color",               SensorHTColor.class);
    // manager.register("LEGO Light",             SensorLEGOLight.class);
    // manager.register("LEGO Touch",             SensorLEGOTouch.class);
    // manager.register("MR Color",               SensorMRColor.class);
    // manager.register("MR Gyro",                SensorMRGyro.class);
    // manager.register("MR IR Seeker",           SensorMRIrSeeker.class);
    // manager.register("MR ODS",                 SensorMROpticalDistance.class);

    //  Concept Samples
    // manager.register("Null Op",                ConceptNullOp.class);
    // manager.register("Compass Calibration",    ConceptCompassCalibration.class);
    // manager.register("DIM as Indicator",       ConceptDIMAsIndicator.class);
    // manager.register("I2C Address Change",     ConceptI2cAddressChange.class);
    // manager.register("Ramp Motor Speed",       ConceptRampMotorSpeed.class);
    // manager.register("Scan Servo",             ConceptScanServo.class);
    // manager.register("Telemetry",              ConceptTelemetry.class);
    // manager.register("Vuforia Navigation",     ConceptVuforiaNavigation.class);
  }
}
