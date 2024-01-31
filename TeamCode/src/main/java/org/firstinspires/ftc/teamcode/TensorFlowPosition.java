/* Copyright (c) 2019 FIRST. All rights reserved.
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

//import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


@TeleOp(name = "TensorFlow Posicao", group = "Concept")
public class TensorFlowPosition extends LinearOpMode {
    private IMU iMU;
    private EventLoopManager eventLoopManager;
    private TeamRobot robot;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
//    Telemetry telemetry;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private Map<String, String> chassisMotorsNames  = new HashMap<String, String>() {{
        put("ef", "esquerdaFrente");
        put("ed", "esquerdaTras");
        put("df", "direitaFrente");
        put("dt", "direitaTras");
    }};
    public TensorFlowPosition() {
        DcMotor[] clawMotorInstances = new DcMotor[]{null, null, null, null};
        HashMap<String, DcMotor> clawMotorHashMap = new HashMap<>();
        String[] chassisMotorLabels = new String[]{"ef", "ed", "df", "dt"};
        DcMotor[] chassisMotorInstances = new DcMotor[]{null, null, null, null};
        HashMap<String, DcMotor> chassiMotorHashMap = new HashMap<>();
        for (int i = 0; i < chassisMotorLabels.length; i++) {
            chassisMotorInstances[i] = hardwareMap.get(DcMotor.class, chassisMotorLabels[i]);
            String motorKey = this.chassisMotorsNames.get(chassisMotorLabels[i]);
            chassiMotorHashMap.put(motorKey, chassisMotorInstances[i]);
        }
        this.iMU = hardwareMap.get(IMU.class, "iMU");
        this.robot = new TeamRobot(chassiMotorHashMap, clawMotorInstances, iMU);
    }

    @Override
    public void runOpMode() {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
        initTfod();
//        telemetry = dashboard.getTelemetry();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private static final String TFOD_MODEL_ASSET = "duckDetector2.tflite";
    private static final String[] LABELS = {
            "Duck",
    };

    private void initTfod() {

        // Define the labels recognized in the model for TFOD (must be in training order!)

        // Create the TensorFlow processor the easy way.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }
    }   // end method initTfod()

    private void getTeamPropPosition() {
        double teamPropPosition;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            teamPropPosition = (recognition.getLeft() + recognition.getRight()) / 2;

            if (teamPropPosition > 300 && teamPropPosition <= 375) {
                telemetry.addData("Pixel no lado ", "meio");
                this.robot.move("forward", 2000);
                // IMPLEMENTAR ESTRATÉGIA PARA OCASIÃO "Pixel no lado: meio"
            } else if (teamPropPosition <= 300) {
                telemetry.addData("Pixel no lado ", "esquerdo");
                this.robot.move("left", 2000);
                // IMPLEMENTAR ESTRATÉGIA PARA OCASIÃO "Pixel no lado: esquerdo"
            } else {
                telemetry.addData("Pixel no lado ", "direito");
                this.robot.move("right", 2000);
                // IMPLEMENTAR ESTRATÉGIA PARA OCASIÃO "Pixel no lado: direito"
            }
        }
    }

    public static void moveMotor(DcMotor motor, String direction, float length) {

    }

    public enum mv {  //move direction
        fwrd("forward"),
        bwrd("backward"),
        stp("stop");
        public final String label;
        private mv(String label) {
            this.label = label;
        }
    }
    class TeamRobot {
        DcMotor motorEsquerdaFrente;
        DcMotor motorEsquerdaTras;
        DcMotor motorDireitaFrente;
        DcMotor motorDireitaTras;
        DcMotor motorGarra1;
        DcMotor motorGarra2;
        public TeamRobot(HashMap<String, DcMotor> chassisMotors, DcMotor[] clawMotors, IMU iMUSensor) {
            this.motorEsquerdaFrente = chassisMotors.get("esquerdaFrente");
            this.motorEsquerdaTras = chassisMotors.get("esquerdaTras");
            this.motorDireitaFrente = chassisMotors.get("direitaFrente");
            this.motorDireitaTras = chassisMotors.get("direitaTras");
        }

        public void move(String direction, float length) {
            switch (direction) {
                case "forward":
                    this.moveForward(length);
                    break;
                case "backward":
                    this.moveBackward(length);
                    break;
                case "left":
                    this.moveLeft(length);
                    break;
                case "right":
                    this.moveRight(length);
                    break;
                default:
                    break;
            }
        }
        private void moveForward(float length) {
            Enum[] chMotorsDirectionsMap = {mv.fwrd, mv.bwrd, mv.fwrd, mv.bwrd};
            this.applyDirectionMapToChassis(chMotorsDirectionsMap, length);
        }

        private void moveBackward(float length) {
            Enum[] chMotorsDirectionsMap = {mv.bwrd, mv.fwrd, mv.bwrd, mv.fwrd};
            this.applyDirectionMapToChassis(chMotorsDirectionsMap, length);
        }

        private void moveLeft(float length) {
            Enum[] chMotorsDirectionsMap = {mv.fwrd, mv.fwrd, mv.bwrd, mv.bwrd};
            this.applyDirectionMapToChassis(chMotorsDirectionsMap, length);
        }

        private void moveRight(float length) {
            Enum[] chMotorsDirectionsMap = {mv.bwrd, mv.bwrd, mv.fwrd, mv.fwrd};
            this.applyDirectionMapToChassis(chMotorsDirectionsMap, length);
        }

        private void applyDirectionMapToChassis(Enum[] chMotorsDirectionsMap, float length) {
            moveMotor(this.motorEsquerdaFrente, String.valueOf(chMotorsDirectionsMap[0]), length);
            moveMotor(this.motorDireitaFrente, String.valueOf(chMotorsDirectionsMap[1]), length);
            moveMotor(this.motorEsquerdaTras, String.valueOf(chMotorsDirectionsMap[2]), length);
            moveMotor(this.motorDireitaTras, String.valueOf(chMotorsDirectionsMap[3]), length);
        }

    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    double x;
    double y;

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2;
            y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Lateral distance (L) (R)", "%.2f / %.2f", recognition.getLeft(), recognition.getRight());
            telemetry.addData("- Vertical distance (U) (D)", "%.2f / %.2f", recognition.getTop(), recognition.getBottom());
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x > 300 && x <= 375) {
                telemetry.addData("Pixel no lado ", "meio");
            } else if (x <= 300) {
                telemetry.addData("Pixel no lado ", "esquerdo");
            } else {
                telemetry.addData("Pixel no lado ", "direito");
            }
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
