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

package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.AdrianControls.VuforiaStuff;
import org.firstinspires.ftc.teamcode.AdrianControls.AdrianMecanumControls;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;
//import com.arcrobotics.ftclib.controller;
//import com.acrobotics.

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

@Autonomous(name="AdrianTestVuforia", group="Linear Opmode")
//@Disabled
public class AdrianTestVuforia extends AdrianMecanumControls {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Encoder Int Values
    private int RightEncoderValue;
    private int LeftEncoderValue;
    private int SideEncoderValue;
    private VuforiaLocalizer vuforia;
    public VuforiaStuff vuforiaStuff;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Initialize Hardware( see AdrianMecanumControls)
        initializeHardware();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaStuff = new VuforiaStuff(vuforia);
        // Wait for the game to start (driver presses PLAY)

        MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);
        //drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //drive.elbowServo.setPosition(1.0);
        //sleep(1000);
        //drive.boxServo.setPosition(0.2);
        //sleep(1000);
        //drive.handServo.setPosition(0.3);

        waitForStart();
        runtime.reset();

        /*
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.ArmMotor.setTargetPosition(drive.ArmMotor.getCurrentPosition() - 400);
        drive.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.ArmMotor.setPower(1);
        while (drive.ArmMotor.isBusy()){

        }

         */

        //  sleep(500);
        //drive.elbowServo.setPosition(0.0);
      //  sleep(1500);

        //drive.handServo.setPosition(0.0);
        //sleep(500);
        //drive.boxServo.setPosition(1.0);
        //sleep(1500);
        //drive.boxServo.setPosition(0.3);




        VuforiaStuff.capElementPositionData posData = null;
        posData = vuforiaStuff.vuforiascan(true, true);
        double distanceToDropOffSkystone = 0;
        double distanceBackToCenterLine = 0;
        double distanceBackToSecondStone = 0;
        boolean turnOnlyOneAtIntake = false;
        VuforiaStuff.capElementPos pos = null;
        pos = posData.capElementPosition;
      drive.ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  /*     //   drive.ArmMotor.setPower(0.176);
         double kSVolts = 1;
        double kCosVolts = 1;
        double kVVoltSecondPerRad = 0.5;
        double kAVoltSecondSquaredPerRad = 0.1;
        ArmFeedforward feedForward = new ArmFeedforward (kSVolts,kCosVolts,kVVoltSecondPerRad,kAVoltSecondSquaredPerRad);
        ElapsedTime timerForPid = new ElapsedTime();
        drive.ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int initialPosition = drive.ArmMotor.getCurrentPosition();
        drive.ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        timerForPid.reset();
        while(Math.abs(drive.ArmMotor.getCurrentPosition() - 180)> 20 && timerForPid.seconds()<2)
        {
            double powerToApply = Math.abs(drive.ArmMotor.getCurrentPosition() - 180) * 0.8/Math.abs(initialPosition-180);
            drive.ArmMotor.setVelocity(feedForward.calculate(1,0.5,0.35));

        }

*/

        if(pos == VuforiaStuff.capElementPos.RIGHT){
            drive.ArmLifter(3,4);
            drive.ArmMotor.setPower(0.176);
            telemetry.addData("FinalPosition", drive.ArmMotor.getCurrentPosition());
            telemetry.addData("powervaluefinal ", drive.ArmMotor.getPower());
            telemetry.update();
            sleep(2000);

        }
        if(pos == VuforiaStuff.capElementPos.CENTER){
            drive.ArmLifter(2,4);
            drive.ArmMotor.setPower(0.176);
            telemetry.addData("FinalPosition", drive.ArmMotor.getCurrentPosition());
            telemetry.addData("powervaluefinal ", drive.ArmMotor.getPower());
            telemetry.update();
            sleep(2000);

        }
       if(pos == VuforiaStuff.capElementPos.LEFT){
            drive.ArmLifter(1,4);
            drive.ArmMotor.setPower(0.176);
            telemetry.addData("FinalPosition", drive.ArmMotor.getCurrentPosition());
            telemetry.addData("powervaluefinal ", drive.ArmMotor.getPower());
            telemetry.update();
            sleep(2000);


        }


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            telemetry.addData("Position", pos);
            telemetry.addData("LeftYellowCount", posData.yellowCountLeft);
            telemetry.addData("CenterYellowCount", posData.yellowCountCenter);
            telemetry.addData("RightYellowCount", posData.yellowCountRight);
            telemetry.addData("ArmCurrentPosition", drive.ArmMotor.getCurrentPosition());
  //          telemetry.addData("powervaluefinal ", drive.ArmMotor.getPower());
//            telemetry.addData("powervaluefinal ", feedForward.calculate(-1,0.25));



            telemetry.update();

            //telemetry.addData("FilePath", path);

            //finding encoder values
/*            RightEncoderValue = GetRightEncoderValue();
            LeftEncoderValue = GetLeftEncoderValue();
            SideEncoderValue = GetSideEncoderValue();

  // grid mecanum movment function( see Adrian Mecanum Controls)
            GridMecanumMovement(-1,1,0.2);



            telemetry.addData("Right Encoder Value", RightEncoderValue);
            telemetry.addData("LeftEncoderValue", LeftEncoderValue);
            telemetry.addData("SideEncoderValue", SideEncoderValue);
            telemetry.update();
         telemetry.update();
    */                }
        }

    }

