package org.firstinspires.ftc.teamcode.Components;

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

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.HashMap;
import java.util.List;

public class TensorFlow {

    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
    private final float mmPerInch = 25.4f;
    final float halfField = 72 * mmPerInch;
    final float quarterField = 36 * mmPerInch;
    final float mmTargetHeight = 5.75f * mmPerInch;
    final float halfTile         = 12 * mmPerInch;
    final float oneAndHalfTile   = 36 * mmPerInch;
    private LedColor led_bank =  null;
    final String VUFORIA_KEY =
            "ATUOrmn/////AAABmVLVlWBtWUpnh9+EekIwR4lmMDXtnMrh/37lRyh+1m4oZJv1ANDvpS7D/Es9GNQ0wAkJ4YOHVWFjjsE5ptAFY2NRCAAwEY4VtvXEvSr3j/a0WR54dNfoCHRsnEaL5oQu25MoyOo7VrmhkE3xb2J9cNbsJzeqNaZWdQQpHkrgzEotos4i2tf/z+IMQxQ5nwH7Daiar93yoFv6FKeTh9MfI3bxVKR0nF+vrMzmNPC6YLk3yjqAKLqSgAvV0t07MBz9BjT2r58njS6qCo2U1H3sQXBlUcMdeKi4iclQaM+Oac+mZrzrhMvSEW7gC9mDhoL8l3zf2yMLPV9oGtnirNWn7ov/mupDtDecOUI4MPDNi9dt";
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    public TensorFlow(LinearOpMode opMode, LedColor ledBank) {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "BackWebcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0 / 9.0);
        }
        led_bank=ledBank;
    }

    private int mode(int[] array) {
        HashMap<Integer, Integer> hm = new HashMap<Integer, Integer>();
        int max = 1;
        int temp = 0;

        for (int i = 0; i < array.length; i++) {

            if (hm.get(array[i]) != null) {

                int count = hm.get(array[i]);
                count++;
                hm.put(array[i], count);

                if (count > max) {
                    max = count;
                    temp = array[i];
                }
            } else
                hm.put(array[i], 1);
        }
        return temp;
    }

    public int DuckTest(LinearOpMode opMode, float cameraX, float cameraY) {
        int i = 0;
        int[] a = {0, 0, 0, 0};
        while(!opMode.isStarted()) {
            boolean detected=false;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                for (int j = 0; j < 4; j++) {
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        detected=true;
                        i = 0;
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            i=0;
                            opMode.telemetry.addData("Position: ","0");
                        }
                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLeft() > -50 && recognition.getLeft() < 50 || recognition.getRight() > 50 && recognition.getRight() < 150){
//                                opMode.telemetry.addData("Position", "1");
//                            }
//                            if (recognition.getLeft() > 200 && recognition.getLeft() < 350 || recognition.getRight() > 300 && recognition.getRight() < 400){
//                                opMode.telemetry.addData("Position", "2");
//                            }
//                            if (recognition.getLeft() > 500 && recognition.getLeft() < 575 || recognition.getRight() > 600 && recognition.getRight() < 675){
//                                opMode.telemetry.addData("Position", "3");
//                            }
                            //below is detection using inches
                            if (recognition.getLabel() == "Duck") {
                                float[] position = positionProcessor(opMode, recognition.getTop(), recognition.getLeft(), recognition.getBottom(), recognition.getRight(), cameraX, cameraY);
                                if (position[0] > -13 && position[0] < -7) {
                                    i = 1;
                                    opMode.telemetry.addData("Position", "1");
                                } else if (position[0] > -3.5 && position[0] < 5) {
                                    i = 2;
                                    opMode.telemetry.addData("Position", "2");
                                } else {
                                    opMode.telemetry.addData("Position", "0");
                                    i = 0;
                                }

                                opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                            }
                        }
                        if(detected) {
                            if(mode(a)==0) {
                                led_bank.LedGreen(1);
                                led_bank.LedOff(2);
                                led_bank.LedOff(3);
                                led_bank.LedOff(4);
                            }
                            else if(mode(a)==1) {
                                led_bank.LedGreen(1);
                                led_bank.LedGreen(2);
                                led_bank.LedOff(3);
                                led_bank.LedOff(4);
                            }
                            else if(mode(a)==2) {
                                led_bank.LedGreen(1);
                                led_bank.LedGreen(2);
                                led_bank.LedGreen(3);
                                led_bank.LedOff(4);
                            }
                        }
                        else{
                            led_bank.LedOff(4);
                            led_bank.LedOff(3);
                            led_bank.LedOff(2);
                            led_bank.LedOff(1);
                        }
                        a[j] = i;
                        opMode.telemetry.update();
                    }
                }
            }
        }
        led_bank.LedGreen(mode(a)+1);
        return mode(a);
    }
    public int ElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        int i = 0;
        int[] a = {0, 0, 0, 0};
        while(!opMode.isStarted()) {
            boolean detected=false;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                for (int j = 0; j < 4; j++) {
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        i = 0;
                        detected=true;
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            i=0;
                            opMode.telemetry.addData("Position: ","0");
                        }

                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLeft() > -50 && recognition.getLeft() < 50 || recognition.getRight() > 50 && recognition.getRight() < 150){
//                                opMode.telemetry.addData("Position", "1");
//                            }
//                            if (recognition.getLeft() > 200 && recognition.getLeft() < 350 || recognition.getRight() > 300 && recognition.getRight() < 400){
//                                opMode.telemetry.addData("Position", "2");
//                            }
//                            if (recognition.getLeft() > 500 && recognition.getLeft() < 575 || recognition.getRight() > 600 && recognition.getRight() < 675){
//                                opMode.telemetry.addData("Position", "3");
//                            }
                            //below is detection using inches
                            if (recognition.getLabel() == "Ball" || recognition.getLabel() == "Cube") {
                                float[] position = positionProcessor(opMode, recognition.getTop(), recognition.getLeft(), recognition.getBottom(), recognition.getRight(), cameraX, cameraY);
                                if (position[0] > -11 && position[0] < -4) {
                                    i = 1;
                                    opMode.telemetry.addData("Position", "1");
                                } else if (position[0] > -2 && position[0] < 2) {
                                    i = 2;
                                    opMode.telemetry.addData("Position", "2");
                                } else {
                                    opMode.telemetry.addData("Position", "0");
                                    i = 0;
                                }

                                opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                            }
                        }
                        if(detected) {
                            if(mode(a)==0) {
                                led_bank.LedGreen(1);
                                led_bank.LedOff(2);
                                led_bank.LedOff(3);
                                led_bank.LedOff(4);
                            }
                            else if(mode(a)==1) {
                                led_bank.LedGreen(1);
                                led_bank.LedGreen(2);
                                led_bank.LedOff(3);
                                led_bank.LedOff(4);
                            }
                            else if(mode(a)==2) {
                                led_bank.LedGreen(1);
                                led_bank.LedGreen(2);
                                led_bank.LedGreen(3);
                                led_bank.LedOff(4);
                            }
                        }
                        else{
                            led_bank.LedOff(4);
                            led_bank.LedOff(3);
                            led_bank.LedOff(2);
                            led_bank.LedOff(1);
                        }
                        a[j] = i;
                        opMode.telemetry.update();
                    }
                }
            }
        }
        led_bank.LedGreen(mode(a)+1);
        return mode(a);
    }
    public int RedElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        int i = 0;
        int[] a = {0, 0, 0, 0};
        while(!opMode.isStarted()) {
            boolean detected = false;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                for (int j = 0; j < 4; j++) {
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        i = 0;
                        detected=true;
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            i=0;
                            opMode.telemetry.addData("Position: ","0");
                        }

                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLeft() > -50 && recognition.getLeft() < 50 || recognition.getRight() > 50 && recognition.getRight() < 150){
//                                opMode.telemetry.addData("Position", "1");
//                            }
//                            if (recognition.getLeft() > 200 && recognition.getLeft() < 350 || recognition.getRight() > 300 && recognition.getRight() < 400){
//                                opMode.telemetry.addData("Position", "2");
//                            }
//                            if (recognition.getLeft() > 500 && recognition.getLeft() < 575 || recognition.getRight() > 600 && recognition.getRight() < 675){
//                                opMode.telemetry.addData("Position", "3");
//                            }
                            //below is detection using inches
                            if (recognition.getLabel() == "Ball") {
                                float[] position = positionProcessor(opMode, recognition.getTop(), recognition.getLeft(), recognition.getBottom(), recognition.getRight(), cameraX, cameraY);
                                if (position[0] > -11 && position[0] < -4) {
                                    i = 1;
                                    opMode.telemetry.addData("Position", "1");
                                } else if (position[0] > -3 && position[0] < 2) {
                                    i = 2;
                                    opMode.telemetry.addData("Position", "2");
                                } else {
                                    opMode.telemetry.addData("Position", "0");
                                    i = 0;
                                }

                                opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                            }
                        }
                        if(detected) {
                            if(mode(a)==0) {
                                led_bank.LedGreen(1);
                                led_bank.LedOff(2);
                                led_bank.LedOff(3);
                                led_bank.LedOff(4);
                            }
                            else if(mode(a)==1) {
                                led_bank.LedGreen(1);
                                led_bank.LedGreen(2);
                                led_bank.LedOff(3);
                                led_bank.LedOff(4);
                            }
                            else if(mode(a)==2) {
                                led_bank.LedGreen(1);
                                led_bank.LedGreen(2);
                                led_bank.LedGreen(3);
                                led_bank.LedOff(4);
                            }
                        }
                        else{
                            led_bank.LedOff(4);
                            led_bank.LedOff(3);
                            led_bank.LedOff(2);
                            led_bank.LedOff(1);
                        }
                        a[j] = i;
                        opMode.telemetry.update();
                    }
                }
            }
        }
        return mode(a);
    }
    public int NathanRedElemTest(LinearOpMode opMode, float cameraX, float cameraY) {
        int i = 0;
        int[] a = {0, 0, 0, 0};
        while(!opMode.isStarted()) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                for (int j = 0; j < 4; j++) {
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        i = 0;
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            i=2;
                            opMode.telemetry.addData("Position: ","2");
                        }

                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLeft() > -50 && recognition.getLeft() < 50 || recognition.getRight() > 50 && recognition.getRight() < 150){
//                                opMode.telemetry.addData("Position", "1");
//                            }
//                            if (recognition.getLeft() > 200 && recognition.getLeft() < 350 || recognition.getRight() > 300 && recognition.getRight() < 400){
//                                opMode.telemetry.addData("Position", "2");
//                            }
//                            if (recognition.getLeft() > 500 && recognition.getLeft() < 575 || recognition.getRight() > 600 && recognition.getRight() < 675){
//                                opMode.telemetry.addData("Position", "3");
//                            }
                            //below is detection using inches
                            if (recognition.getLabel() == "Ball" || recognition.getLabel() == "Cube") {
                                float[] position = positionProcessor(opMode, recognition.getTop(), recognition.getLeft(), recognition.getBottom(), recognition.getRight(), cameraX, cameraY);
                                if (position[0] > -11 && position[0] < -4) {
                                    i = 0;
                                    opMode.telemetry.addData("Position", "0");
                                } else if (position[0] > -2 && position[0] < 2) {
                                    i = 1;
                                    opMode.telemetry.addData("Position", "1");
                                } else {
                                    opMode.telemetry.addData("Position", "2");
                                    i = 2;
                                }

                                opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                            }
                        }
                        a[j] = i;
                        opMode.telemetry.update();
                    }
                }
            }
        }
        return mode(a);
    }
    public float[] positionProcessor(LinearOpMode opMode, float top, float left, float bottom, float right, float bigboyX, float bigboyY) {
        float[] position = {0,0};
        float gamerconst = 29f/24*27; // converts pixels to inches for distance(needs tuning)
        float angleconst = 12.25f; // converts pixels to inches for angle(needs tuning)
        float size=(float)sqrt(pow((top-bottom),2)+pow((left-right),2));
        float dist = ((float)sqrt(800*800+500*500)-size)/gamerconst;
        float[] midpoynt = {(right+left)/2, (top+bottom)/2};
        float angule = (float)(midpoynt[0]-400)/angleconst;
        float hypotenuse = (float)sqrt(pow(midpoynt[1]-250, 2)+pow(midpoynt[0]-400,2));
        float opposite = (float)sin(angule*PI/180)*dist-bigboyX; // the x
        float adjacent = (float)cos(angule*PI/180)*dist-bigboyY; // the y
        position[1] = adjacent;
        position[0] = opposite;
        return position;
    }
}