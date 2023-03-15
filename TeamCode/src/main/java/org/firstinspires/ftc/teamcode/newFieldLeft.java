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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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

@Autonomous (name= "1 Cone Field Left", group = "Pushbot")
public class newFieldLeft extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String VUFORIA_KEY =
            "ASxSfhX/////AAABmWcpvgdyP053gmhPvX7/JZ5yybQKAVFnqMk+WjYvbuuiectzmcdkuftxSIgVawrOZ7CQOqdHzISXbHCAom4FhIzrDceJIIEGozFWpgAu5dUKc3q843Hd3x875VOBf8B7DlD7g9TgqxqgQRw9coEUBBeEJqy2KGy4NLPoIKLdiIx8yxSWm7SlooFSgmrutF/roBtVM/N+FhY6Sgdy9fgWssccAhd2IxdYllAaw4s1oC1jqtwbjIsdjNVogmwwXdTmqiKHait1PFyF2FDNfKi+7qs4Mc6KbvXD2FHA6RljkcN5Oo080o2QSVCzDuQtJeagh/CglB2PcatFWnebiWN+a43kEdrUaY+uq0YQ8m9IRBWE";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private TeamMarkerDetector detector;
    private Constants.SamplingLocation samplingLocation = Constants.SamplingLocation.RIGHT;

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
        HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
        robot.init(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        telemetry.addData("pre int", "wowie");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", " id", hardwareMap.appContext.getPackageName());
        telemetry.addData("got past the int", "big wowie");
        telemetry.update();
        detector = new TeamMarkerDetector(cameraMonitorViewId);

        telemetry.addData("we got here", "before the waitForStart(); ");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        //List<Recognition> updatedRecognitions = tfod.getRecognitions();
        waitForStart();
        double currenttime = runtime.seconds();


        TeamMarkerDetector.ColorPreset colorPreset = detector.sample(true);
        sleep(1);

        telemetry.addData(colorPreset.name(), "");
        telemetry.update();
        sleep(1000);
        boolean sawOrange=false;
        boolean sawPurple=false;
        boolean sawGreen =false;


        switch (colorPreset) {
            case PURE_ORANGE:
                telemetry.addData("orange was detected", "");
                telemetry.update();
                sawOrange=true;
                break;
            case PURE_GREEN:
                telemetry.addData("green was detected", "");
                telemetry.update();
                sawGreen=true;
                break;
            case PURE_PURPLE:
                telemetry.addData("purple was detected", "");
                telemetry.update();
                sawPurple=true;
                break;
            case PURE_GRAY:
                telemetry.addData("oh no its gray so sad", "");
                telemetry.update();
                break;
            default:
                telemetry.addData("oh no its nothing! so we are going to set it to orange", "");
                telemetry.update();
                break;
        }
        dropConeOnPole(robot, sawOrange,sawGreen,sawPurple);
        sleep(1000);
    }

    void dropConeOnPole(HardwarePushbot robot,boolean sawOrange,boolean sawGreen, boolean sawPurple){
        Movement movement = new Movement();

        movement.reset(robot);
        movement.Backward(50, robot, 0.5);
        movement.move(robot);

        robot.claw.setPosition(0);

        sleep(1000);

        movement.reset(robot);
        movement.Forward(1050, robot, 0.5);
        movement.move(robot);

        sleep(1200);

        robot.liftLeft.setPower(0.6);
        robot.liftRight.setPower(0.6);

        movement.reset(robot);
        movement.Left(2100, robot, 0.5);
        movement.move(robot);

        sleep(4000);
        robot.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.liftLeft.setPower(0.0);
        robot.liftRight.setPower(0.0);
        //sleep(100);
        robot.claw.setPosition(0.45);

        sleep(2500);

        if (sawOrange==true){
            movement.reset(robot);
            movement.Right(700, robot, 0.5);
            movement.move(robot);
            sleep(2000);
            movement.reset(robot);
            movement.Backward(2200, robot, 0.5);
            movement.move(robot);
        }
        if (sawGreen==true){
            movement.reset(robot);
            movement.Left(600, robot, 0.5);
            movement.move(robot);
            sleep(2000);
            movement.reset(robot);
            movement.Backward(1100, robot, 0.5);
            movement.move(robot);
        }
        if (sawPurple==true){
            movement.reset(robot);
            movement.Left(600, robot, 0.5);
            movement.move(robot);
            sleep(2000);
        }
        sleep(10000);

       /* movement.reset(robot);
        movement.Rrotate(180, robot, 0.5);
        movement.move(robot);*/
    }

}