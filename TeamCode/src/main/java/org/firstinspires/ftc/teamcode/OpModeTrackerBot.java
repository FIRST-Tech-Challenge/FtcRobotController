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

import static org.firstinspires.ftc.teamcode.PipeTrackerBot.getAmountDetectedBoxes;
import static org.firstinspires.ftc.teamcode.PipeTrackerBot.getAvgXBoxes;
import static org.firstinspires.ftc.teamcode.PipeTrackerBot.getAvgYBoxes;
import static org.firstinspires.ftc.teamcode.PipeTrackerBot.getLastCurrentBoxesDifference;
import static org.firstinspires.ftc.teamcode.Variables.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DriveMethods;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;





/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="OpModeTrackerBot", group="Linear Opmode")

public class OpModeTrackerBot extends DriveMethods {

    // Declare OpMode members.

    OpenCvCamera camera;
    double leftY1;
    double leftX1;
    double rightX1;
    double targetX = 160;
    double differenceX = 0;
    double differenceBoxes = 0;
    double multiplierX = 0.0025;
    double multiplierBoxes = 0.02;
    double targetBoxes = 8;

//    double frontleftpower = 0; //The lowercase variables represent the
//    double backleftpower = 0;
//    double frontrightpower = 0;
//    double backrightpower = 0;


    double FrontLeftPower = 0;
    double BackLeftPower = 0;
    double FrontRightPower = 0;
    double BackRightPower = 0;




    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    private final String replacement = "";
    private final String target = " seconds";



    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        PipeTrackerBot pipeTrackerBot = new PipeTrackerBot(telemetry);
        camera.setPipeline(pipeTrackerBot);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);


            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });



        initMotorsBlue();
        calibrateNavXIMU();

        waitForStart();

        runtime.reset();


        telemetry.addData("Status", "Initialized");
        telemetry.update();






        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            String stringStateRuntime = timer.toString();
            String processed = stringStateRuntime.replace(target,replacement);
            Double doubleStateRuntime = Double.parseDouble(processed);




            if(gamepad1.right_bumper && doubleStateRuntime > 0.2){
                targetBoxes++;
                timer.reset();
            }
            if(gamepad1.left_bumper && doubleStateRuntime > 0.2){
                targetBoxes--;
                timer.reset();
            }

            getAvgXBoxes();
            getAvgYBoxes();







            differenceX = targetX - getAvgXBoxes();
            differenceBoxes = targetBoxes - getAmountDetectedBoxes();



            leftY1 = -gamepad1.left_stick_y;
            leftX1 = gamepad1.left_stick_x;
            rightX1 = gamepad1.right_stick_x;

            if(getAvgXBoxes() == 0){
                differenceX = 0;
            }
            if(getAmountDetectedBoxes() == 0){
                differenceBoxes = 0;
            }

            FrontLeftPower = ((leftY1 + leftX1 + rightX1) - differenceX*multiplierX + differenceBoxes*multiplierBoxes);
            BackLeftPower = ((leftY1 - leftX1 + rightX1) - differenceX*multiplierX + differenceBoxes*multiplierBoxes);
            FrontRightPower = ((leftY1 - leftX1 - rightX1) + differenceX*multiplierX + differenceBoxes*multiplierBoxes);
            BackRightPower = ((leftY1 + leftX1 - rightX1) + differenceX*multiplierX + differenceBoxes*multiplierBoxes);


//            if(Math.abs(getLastCurrentBoxesDifference()) > 10){
//                FrontLeftPower = 0;
//                BackLeftPower = 0;
//                FrontLeftPower = 0;
//                BackRightPower = 0;
//            }


            motorFL.setPower(FrontLeftPower);
            motorBL.setPower(BackLeftPower);
            motorFR.setPower(FrontRightPower);
            motorBR.setPower(BackRightPower);





            telemetry.addLine("doubleStateRuntime: " + doubleStateRuntime);
            telemetry.addLine("Average Coordinate is (" + getAvgXBoxes() + ", " + getAvgYBoxes() + ")");
            telemetry.addLine("MultiplierX*DifferenceX is: " + differenceX*multiplierX);
            telemetry.addLine("MultiplierBoxes*DifferenceBoxes is: " + differenceBoxes*multiplierBoxes);
            telemetry.addLine("DifferenceX is: " + differenceX);
            telemetry.addLine("Current Boxes:  " + getAmountDetectedBoxes());
            telemetry.addLine("Target Boxes:  " + targetBoxes);
            telemetry.addLine("Difference Boxes:  " + differenceBoxes);
            telemetry.addLine("LastCurrentBoxesDifference: " + getLastCurrentBoxesDifference());



            telemetry.addLine(runtime.toString());
            telemetry.update();


        }
        camera.stopStreaming();
    }



}

