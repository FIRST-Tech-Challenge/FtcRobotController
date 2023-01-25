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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Constants.SamplingLocation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import java.util.List;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 * hi
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="redduckspin", group="Pushbot")
public class redduckspin extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private ElapsedTime runtime = new ElapsedTime();
    private static final String[] LABELS = {
        //"Ball",
        //"Cube",
        "Duck",
        "Marker"
    };
    private static final String VUFORIA_KEY =
        "ASxSfhX/////AAABmWcpvgdyP053gmhPvX7/JZ5yybQKAVFnqMk+WjYvbuuiectzmcdkuftxSIgVawrOZ7CQOqdHzISXbHCAom4FhIzrDceJIIEGozFWpgAu5dUKc3q843Hd3x875VOBf8B7DlD7g9TgqxqgQRw9coEUBBeEJqy2KGy4NLPoIKLdiIx8yxSWm7SlooFSgmrutF/roBtVM/N+FhY6Sgdy9fgWssccAhd2IxdYllAaw4s1oC1jqtwbjIsdjNVogmwwXdTmqiKHait1PFyF2FDNfKi+7qs4Mc6KbvXD2FHA6RljkcN5Oo080o2QSVCzDuQtJeagh/CglB2PcatFWnebiWN+a43kEdrUaY+uq0YQ8m9IRBWE";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    //    /**
//     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
//     * Detection engine.
//     */
    private TFObjectDetector tfod;

    private TeamMarkerDetector detector;

    private SamplingLocation samplingLocation = SamplingLocation.RIGHT;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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
//        vuforiaLocalizer.enableConvertFrameToBitmap();
//        vuforiaLocalizer.setFrameQueueCapacity(1);
//    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        // initVuforia();
//        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
//        if (tfod != null) {
//            tfod.activate();

//             // The TensorFlow software will scale the input images from the camera to a lower resolution.
//             // This can result in lower detection accuracy at longer distances (> 55cm or 22").
//             // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
//             // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
//             // should be set to the value of the images used to create the TensorFlow Object Detection model
//             // (typically 16/9).
//            tfod.setZoom(1.0, 16 / 9.0);
//         }


            /* Declare OpMode members. */
            HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
            ElapsedTime     runtime = new ElapsedTime();


            final double     FORWARD_SPEED = 0.3;
            final double     TURN_SPEED    = 0.3;
            int markerPosition = 3;
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
//        List<Recognition> updatedRecognitions = tfod.getRecognitions();


            // For Sampling. Note: change imageSavingEnabled to see what the Detector is sampling against
            telemetry.addData("pre int", "wowie");
        telemetry.update();
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId"," id", hardwareMap.appContext.getPackageName());
            telemetry.addData("got past the int", "big wowie");
        telemetry.update();
            detector = new TeamMarkerDetector(cameraMonitorViewId);

            telemetry.addData("we got here", "before the waitForStart(); ");
        telemetry.update();
            waitForStart();

        double currenttime = runtime.seconds();



            // Perform sampling
           /* boolean thisSample = true;
            samplingLocation = detector.sample(thisSample);
            sleep(1);

            switch (samplingLocation) {
                case CENTER:
                    telemetry.addData("center", "");
                    telemetry.update();
                    break;
                case LEFT:
                    telemetry.addData("left", "");
                    telemetry.update();
                    break;
                case RIGHT:
                    telemetry.addData("right", "");
                    telemetry.update();
                    break;
            }*/



//red warhouse
        /*robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1300);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(-TURN_SPEED);
        robot.backLeft.setPower(TURN_SPEED);
        sleep(2400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(375);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/
// blue warhouse
        /*robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1300);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(TURN_SPEED);
        robot.frontRight.setPower(-TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        sleep(2400);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(1000);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        robot.frontLeft.setPower(.30);
        robot.frontRight.setPower(.30);
        robot.backLeft.setPower(.30);
        robot.backRight.setPower(.30);
        sleep(375);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/

            //Duck spinning red
        /*robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(560);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(750);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        sleep(2000);

        robot.spinServo.setPower(0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(0.40);
        robot.frontRight.setPower(-0.40);
        robot.backLeft.setPower(-0.40);
        robot.backRight.setPower(0.40);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(250);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
//new stuff
        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(725);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.shuteServo.setPower(0.3);
        sleep(1450);
        robot.shuteServo.setPower(0);

        robot.gatherServo.setPower(-0.4);
        sleep(2200);
        robot.gatherServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(500);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.shuteServo.setPower(-0.3);
        sleep(1450);
        robot.shuteServo.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(725);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        /*robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(700);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);
        sleep(960);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(1200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(0.25);
        robot.frontRight.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.backRight.setPower(0.25);
        sleep(420);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.spinServo.setPower(-0.4);
        sleep(2600);
        robot.spinServo.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(-0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(-0.30);
        sleep(1120);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(-0.30);
        sleep(200);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /*robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(-0.30);
            sleep(700);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-0.20);
            robot.frontRight.setPower(-0.20);
            robot.backLeft.setPower(-0.20);
            robot.backRight.setPower(-0.20);
            sleep(2000);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.frontLeft.setPower(0);
            sleep(2000);

            robot.spinServo.setPower(0.4);
            sleep(2600);
            robot.spinServo.setPower(0);

            robot.frontLeft.setPower(-0.40);
            robot.frontRight.setPower(0.40);
            robot.backLeft.setPower(0.40);
            robot.backRight.setPower(-0.40);
            sleep(1030);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(-0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(-0.30);
            sleep(250);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(0.30);
            sleep(300);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(0.30);
            sleep(300);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(0.30);
            sleep(300);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setPower(-0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(-0.30);
            robot.backRight.setPower(0.30);
            sleep(2000);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);


            robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            if(markerPosition == 3) {
                robot.liftLeft.setTargetPosition(-290);
                robot.liftRight.setTargetPosition(-290);
                robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.liftLeft.setPower(0.20);
                robot.liftRight.setPower(0.20);

                sleep(2020);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(350);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.gatherServo.setPower(-0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);

            } else if (markerPosition == 2){

                robot.liftLeft.setTargetPosition(-250);
                robot.liftRight.setTargetPosition(-250);
                robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(80);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setPower(0.30);
                robot.liftRight.setPower(0.30);

                sleep(2020);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(700);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.gatherServo.setPower(-0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(0.30);
                sleep(500);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);

            }else if(markerPosition == 1){
                robot.liftLeft.setTargetPosition(-200);
                robot.liftRight.setTargetPosition(-200);
                robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(80);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setPower(0.30);
                robot.liftRight.setPower(0.30);

                sleep(2020);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(700);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.gatherServo.setPower(-0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(0.30);
                sleep(500);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);

            } else if (markerPosition == 1){
                robot.liftLeft.setTargetPosition(-564);
                robot.liftRight.setTargetPosition(-564);
                robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(60);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setPower(0.30);
                robot.liftRight.setPower(0.30);

                sleep(2020);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(-0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(-0.30);
                sleep(700);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.gatherServo.setPower(-0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.frontLeft.setPower(-0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(-0.30);
                robot.backRight.setPower(0.30);
                sleep(500);
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);
            }

        robot.frontLeft.setPower(-0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(-0.30);
        robot.backRight.setPower(0.30);
        sleep(20);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

            robot.frontLeft.setPower(0.30);
            robot.frontRight.setPower(0.30);
            robot.backLeft.setPower(0.30);
            robot.backRight.setPower(0.30);
            sleep(1100);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            sleep(300);*/

        /*robot.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftPosition -= 120;
        frontRightPosition += 120;
        backLeftPosition += 120;
        backRightPosition -= 120;
        //going out from wall

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);

        sleep(1200);

        frontLeftPosition -= 600;
        frontRightPosition -= 600;
        backLeftPosition -= 600;
        backRightPosition -= 600;
        //moving toward the spinner

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.25);
        robot.frontRight.setPower(0.25);
        robot.backLeft.setPower(0.25);
        robot.backRight.setPower(0.25);

        sleep(1500);

        //robot.spinServo.setPower(0.5);
        robot.frontLeft.setPower(-0.05);
        robot.frontRight.setPower(-0.05);
        robot.backLeft.setPower(-0.05);
        robot.backRight.setPower(-0.05);
        sleep(4300);
        //robot.spinServo.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        frontLeftPosition -= 730;
        frontRightPosition += 730;
        backLeftPosition += 730;
        backRightPosition -= 730;
        //moving sideways to the box

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);

        sleep(1200);


        frontLeftPosition += 420;
        frontRightPosition += 420;
        backLeftPosition += 420;
        backRightPosition += 420;
        //going backwards toward the shipping hub

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.30);
        robot.frontRight.setPower(0.30);
        robot.backLeft.setPower(0.30);
        robot.backRight.setPower(0.30);

        sleep(1200);

        switch (samplingLocation) {
            case CENTER:
                frontLeftPosition -= 1080;
                frontRightPosition += 1080;
                backLeftPosition -= 1080;
                backRightPosition += 1080;
                //spinning toward the shipping

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1200);

                robot.liftLeft.setTargetPosition(-500);
                robot.liftRight.setTargetPosition(-500);
                robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.liftLeft.setPower(0.20);
                robot.liftRight.setPower(0.20);

                sleep(2020);

                frontLeftPosition -= 260;
                frontRightPosition -= 260;
                backLeftPosition -= 260;
                backRightPosition -= 260;
                //going foward toward the shipping hub

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1200);

                //robot.gatherServo.setPower(0.4);
                sleep(2200);
                //robot.gatherServo.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);

                frontLeftPosition -= 250;
                frontRightPosition += 250;
                backLeftPosition -= 250;
                backRightPosition += 250;
                //turing to go into red square

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1500);

                frontLeftPosition += 900;
                frontRightPosition += 900;
                backLeftPosition += 900;
                backRightPosition += 900;
                //going backwards to go into red square

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(2000);

                break;
            case LEFT:
                frontLeftPosition += 690;
                frontRightPosition += 690;
                backLeftPosition += 690;
                backRightPosition += 690;
                //going back toward the shipping

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1200);

                robot.liftLeft.setTargetPosition(-100);
                robot.liftRight.setTargetPosition(-100);
                robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.liftLeft.setPower(0.20);
                robot.liftRight.setPower(0.20);

                sleep(1000);

                robot.gatherServo.setPower(0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                sleep(3000);

                frontLeftPosition -= 1340;
                frontRightPosition -= 1340;
                backLeftPosition -= 1340;
                backRightPosition -= 1340;
                //going back toward the red square

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1200);

                frontLeftPosition -= 120;
                frontRightPosition += 120;
                backLeftPosition += 120;
                backRightPosition -= 120;
                //turing to go into red square

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1500);
                break;

            case RIGHT:
                frontLeftPosition -= 1100;
                frontRightPosition += 1100;
                backLeftPosition -= 1100;
                backRightPosition += 1100;
                //spinning toward the shipping

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1200);

                robot.liftLeft.setTargetPosition(-460);
                robot.liftRight.setTargetPosition(-460);
                robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.liftLeft.setPower(0.20);
                robot.liftRight.setPower(0.20);

                sleep(2020);

                frontLeftPosition -= 420;
                frontRightPosition -= 420;
                backLeftPosition -= 420;
                backRightPosition -= 420;
                //going foward toward the shipping hub

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1200);

                robot.gatherServo.setPower(0.4);
                sleep(2200);
                robot.gatherServo.setPower(0);

                robot.liftLeft.setTargetPosition(0);
                robot.liftRight.setTargetPosition(0);

                frontLeftPosition -= 160;
                frontRightPosition += 160;
                backLeftPosition -= 160;
                backRightPosition += 160;
                //turing to go into red square

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(1500);

                frontLeftPosition += 1000;
                frontRightPosition += 1000;
                backLeftPosition += 1000;
                backRightPosition += 1000;
                //going backwards to go into red square

                robot.frontLeft.setTargetPosition(frontLeftPosition);
                robot.frontRight.setTargetPosition(frontRightPosition);
                robot.backLeft.setTargetPosition(backLeftPosition);
                robot.backRight.setTargetPosition(backRightPosition);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.30);
                robot.frontRight.setPower(0.30);
                robot.backLeft.setPower(0.30);
                robot.backRight.setPower(0.30);

                sleep(2000);

                break;*/
        }



    }
//    /**
//     * Initialize the TensorFlow Object Detection engine.
//     */
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.75f ;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 320;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//    }
    //}

