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

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.skills.RingDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

@TeleOp(name="Ring Rec", group="Robot15173")
//@Disabled
public class RingRecogTest extends LinearOpMode {

    // Declare OpMode members.
    YellowBot robot = new YellowBot();
    private ElapsedTime runtime = new ElapsedTime();

    private static final String VUFORIA_KEY =
            "AZs0syj/////AAABmaxIME6H4k74lx12Yv3gnoYvtGHACOflWi3Ej36sE7Hn86xDafDA3vhzxSOgBtyNIQ1ua6KP2j3I2ScFedVw8n6MJ7PReZQP4sTdc8gHvoy17hD574exMmoUQ3rVUMkgU2fwN2enw2X+Ls2F3BLuCg/A4SBZjzG3cweO+owiKO/2iSIpeC4rBdUnPiTxqPHNa8UOxyncCGV0+ZFXresQm/rK7HbOKB9MEszi8eW2JNyfjTdKozwDxikeDRV7yPvoIhZ5A+mrrC1GgrEzYwNTVHeki2cg4Ea62pYwdscaJ+6IWHlBIDutqmgJu/Os3kAzZkOh0TJ8P3e29Ou4ZczTdDH0oqkPt78Nt4VdYbSbLRCw";


    private boolean ringDetected = false;
    private RingDetector sf = null;


    @Override
    public void runOpMode() {
        try {
            try {
                sf = new RingDetector(this.hardwareMap, telemetry);
//                robot.init(this, this.hardwareMap, telemetry);
            }
            catch (Exception ex){
                telemetry.addData("Init", ex.getMessage());
            }

            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {


                ringDetected = sf.detectRing(30, telemetry,this);
                if(ringDetected) {

                    telemetry.addData("Rec", "Ring Zone: %s", sf.getTargetZone());
                    telemetry.update();
                }
            }
        }
        catch (Exception ex){
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(10000);
        }
        finally {
            if (sf != null){
                sf.stopDetection();
            }
        }
    }

//    protected void initRec(){
//        if (vuforia == null){
//            telemetry.addData("Error", "Unable to start Vuforia");
//        }
//        else {
//            telemetry.addData("Info", "Vuforia initialized");
//
//            try {
//
//                if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//                    initTfod();
//                    telemetry.addData("Info", "TF initialized");
//                } else {
//                    telemetry.addData("Error", "This device is not compatible with TFOD");
//                }
//
//                /**
//                 * Activate TensorFlow Object Detection before we wait for the start command.
//                 * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
//                 **/
//                if (tfod != null) {
//                    tfod.activate();
//                }
//                telemetry.addData("Info", "TF Activated");
//            }
//            catch (Exception ex){
//                telemetry.addData("Error", "Unable to initialize Tensor Flow");
//            }
//        }
//        telemetry.update();
//    }

//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//    }

//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minimumConfidence = 0.8;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SKYSTONE);
//    }

}
