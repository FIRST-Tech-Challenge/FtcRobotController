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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.CameraDetector;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.CupServo;
import org.firstinspires.ftc.teamcode.Functions.MoveAutocorrect2;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name = "Autonomous2022", group = "Concept")
@Disabled
public class Autonomous2022 extends LinearOpMode {

    public CameraDetector cameraDetector;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack,vaccumRight,vaccumLeft, carouselMotor;
    private DcMotorEx armMotorLeft, armMotorRight;
    private CRServo collectorCr;
    private Move move;
    private Rotate rotate;
    private RotationDetector rotationDetector;
    private ArmEncoder armEncoder;
    private Collector collector;
    public VoltageReader voltageReader;
    private MoveAutocorrect2 AutoCorrection;
    private Servo servoSR;
    private CupServo cupServo;


    private int leftBackPos;
    private int rightBackPos;
    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        vaccumRight = hardwareMap.dcMotor.get("VR");
        vaccumLeft = hardwareMap.dcMotor.get("VL");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "AMR");
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "AML");

        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        collectorCr = hardwareMap.crservo.get("CR");
        collector = new Collector(collectorCr);
        cameraDetector = new CameraDetector(vuforia,tfod,hardwareMap.get(WebcamName.class, "Webcam"));
        servoSR = hardwareMap.servo.get("SR");
        cupServo = new CupServo(servoSR);
        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        AutoCorrection = new MoveAutocorrect2(rotationDetector,move,rotate);

//        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
//        voltageReader = new VoltageReader(VS);
//        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//        // first.
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        cameraDetector.initVuforia();
//        cameraDetector.initTfod(tfodMonitorViewId);
//        float duckLeft, duckRight, duckTop, duckBottom;
//
//        // cameraDetector.activateTenserFlow(tfod);
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//        waitForStart();
//        cameraDetector.duckIsDetected(tfod);
//        telemetry.addData("Duck status:", cameraDetector.duckIsDetected(tfod));
//        telemetry.addData(String.format("  left,top (%d)" ), "%.03f , %.03f",
//                cameraDetector.duckValues.left, cameraDetector.duckValues.top);
//        telemetry.addData(String.format("  left,top (%d)" ), "%.03f , %.03f",
//                cameraDetector.duckValues.right, cameraDetector.duckValues.bottom);
//        telemetry.update();
        /**
         * Rotesc robotul la 45 de grade
         */
        waitForStart();

        sleep(200);
        /**
         * Duc robotul in fata 20 cm
         * usage: move.MoveFull(direction), sleep(voltageReader.GetWaitTime(distance, direction)
         * direction : 1 - front, 2 - back, 3 - right, 4 - left
         * sleep = for how long it moves
         * GetWaitTime(distance, direction) distance - in cm(degrees if direction = 3), direction - 1(front,back), 2(slide left, slide right), 3(rotation angle)
         * move.MoveStop(); we have to stop the motors after movint the distance(20cm);
         */
        move.MoveRaw(4, 0.5);
        sleep(200);
        move.MoveStop();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackPos = 0;
        rightBackPos = 0;
        drive(-1400, -1400, 0.7);

        collectorCr.setPower(-1);
        sleep(5000);
        collectorCr.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        drive(6000,6000,0.6);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         */
        //move.MoveFull(2);
        Autocorrect(180,3000,2);
        //sleep(3000);
        move.MoveStop();


        /**
         * Aici trebe sa completezi tu cu bratul ( ii dai putere apoi dai la aspirator invers ca sa scoata cubu apoi
         * te dai un pic in spate si lasi bratu jos presupun)
         */
//        arm.Start(0.5);
//        cupServo.Left();
        /*
        drive(-100, -100, 0.5);

         int Unghi = 60;
        while(opModeIsActive() && rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        drive(3400, 3400, 0.5);

        collector.StartContinuous();
        sleep(5000);
        collector.StopContinous();

        Unghi = 90;
        while(opModeIsActive() && rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(2, rotationDetector.MotorPower(Unghi));
        }
        drive(6400, 6400, 0.5);

         */
    }

    private void Autocorrect(int initialAngle, int time, int direction){
        AutoCorrection.GivenAngle(initialAngle);
        while(time > 0){
            move.MoveFull(direction);
            AutoCorrection.MoveFull(direction);
            time -=1;
            telemetry.addData("Timpul ",time);
            telemetry.update();
        }
        move.MoveStop();
    }
    private void drive(int rightBackTarget, int leftBackTarget, double speed){

//        leftPos += leftTarget;
//        rightPos += rightTarget;
        leftBackPos += leftBackTarget;
        rightBackPos += rightBackTarget;


        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
//        leftMotor.setTargetPosition(leftPos);
//        rightMotor.setTargetPosition(rightPos);
        leftMotor.setTargetPosition(leftBackPos);
        rightMotor.setTargetPosition(rightBackPos);

//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        leftMotor.setPower(speed);
//        rightMotor.setPower(speed);
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()){
            idle();
            //Autocorrect();
        }
    }
}
