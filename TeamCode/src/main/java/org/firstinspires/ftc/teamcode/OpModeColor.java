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

import static org.firstinspires.ftc.teamcode.PipeColor.getHueLower;
import static org.firstinspires.ftc.teamcode.PipeColor.getHueUpper;
import static org.firstinspires.ftc.teamcode.PipeColor.getSaturationLower;
import static org.firstinspires.ftc.teamcode.PipeColor.getSaturationUpper;
import static org.firstinspires.ftc.teamcode.PipeColor.getValueLower;
import static org.firstinspires.ftc.teamcode.PipeColor.getValueUpper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.PipeColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Taank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="ColorDetection", group="Linear Opmode")

public class OpModeColor extends DriveMethods {

    // Declare OpMode members.

    ElapsedTime timer = new ElapsedTime();
    OpenCvCamera camera;
    double setHue = 0;
    double setSaturation = 0;
    double setValue = 0;
    int setHueSeparation = 0;
    int setSaturationSeparation = 0;
    int setValueSeparation = 0;


    private final String replacement = "";
    private final String target = " seconds";




    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam720");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        PipeColor pipeColor = new PipeColor(telemetry, 0 ,0 ,0,0,0,0);
        camera.setPipeline(pipeColor);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480,OpenCvCameraRotation.UPRIGHT);


            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();
        timer.reset();


        telemetry.addData("Status", "Initialized");
        telemetry.update();






        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        setHue = 0;
        setSaturation = 0;
        setValue = 0;
        setHueSeparation = 0;
        setSaturationSeparation = 0;
        setValueSeparation = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            String stringStateRuntime = timer.toString();
            String processed = stringStateRuntime.replace(target,replacement);
            Double doubleStateRuntime = Double.parseDouble(processed);



            if(gamepad1.left_stick_x > 0.2 && doubleStateRuntime > 0.1){
                setSaturation++;
                timer.reset();
            }
            if(gamepad1.left_stick_x < -0.2 && doubleStateRuntime > 0.1){
                setSaturation--;
                timer.reset();
            }
            if(-gamepad1.left_stick_y > 0.2 && doubleStateRuntime > 0.1){
                setHue++;
                timer.reset();
            }
            if(-gamepad1.left_stick_y < -0.2 && doubleStateRuntime > 0.1){
                setHue--;
                timer.reset();
            }
            if(-gamepad1.right_stick_y > 0.2 && doubleStateRuntime > 0.1){
                setValue++;
                timer.reset();
            }
            if(-gamepad1.right_stick_y < -0.2 && doubleStateRuntime > 0.1){
                setValue--;
                timer.reset();
            }
            if(gamepad1.dpad_up && doubleStateRuntime > 0.2){
                setHueSeparation++;
                timer.reset();
            }
            if(gamepad1.dpad_down && doubleStateRuntime > 0.2){
                setHueSeparation--;
                timer.reset();
            }
            if(gamepad1.dpad_right && doubleStateRuntime > 0.2){
                setSaturationSeparation++;
                timer.reset();
            }
            if(gamepad1.dpad_left && doubleStateRuntime > 0.2){
                setSaturationSeparation--;
                timer.reset();
            }
            if(gamepad1.right_bumper && doubleStateRuntime > 0.2){
                setValueSeparation++;
                timer.reset();
            }
            if(gamepad1.left_bumper && doubleStateRuntime > 0.2){
                setValueSeparation--;
                timer.reset();
            }



            pipeColor = new PipeColor(telemetry, setHue, setSaturation, setValue, setHueSeparation, setSaturationSeparation, setValueSeparation);
            camera.setPipeline(pipeColor);

            telemetry.addLine("H_lower: "+ getHueLower());
            telemetry.addLine("H_upper: "+ getHueUpper());
            telemetry.addLine("S_lower: "+ getSaturationLower());
            telemetry.addLine("S_upper: "+ getSaturationUpper());
            telemetry.addLine("V_lower: "+ getValueLower());
            telemetry.addLine("V_upper: "+ getValueUpper());

            telemetry.update();


//            setHue = 0;
//            setHueSeparation = 0;
//            setSaturation = 0;
//            setSaturationSeparation = 0;
//            setValue = 0;
//            setValueSeparation = 0;


        }
        camera.stopStreaming();
    }
}

