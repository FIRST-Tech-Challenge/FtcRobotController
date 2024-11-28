/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor elevatorLift = null;
    private DcMotor intakeSlide = null;
    private Servo intakePincher = null;
    Limelight3A limelight = null;
    GoBildaPinpointDriver odo = null; // Declare OpMode member for the Odometry Computer
    private RevBlinkinLedDriver blinkinLedDriver = null;
    private RevBlinkinLedDriver.BlinkinPattern pattern = null;
    private DigitalChannel allianceButton = null;
    private LED LED_red = null;
    private LED LED_green = null;

    //private Servo   leftHand = null;
    //private Servo   rightHand = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    //public static final double MID_SERVO       =  0.5 ;
    //public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {

        ///GoBilda Odometry Pod Setup
        //Deploy to Control Hub to make Odometry Pod show in hardware selection list
        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //TODO Set Odometry Offsets
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        ///REV LED Setup
        LED_green = myOpMode.hardwareMap.get(LED.class, "LED_green");
        LED_red = myOpMode.hardwareMap.get(LED.class, "LED_red");

        ///Blinkin Setup
        blinkinLedDriver = myOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        allianceButton = myOpMode.hardwareMap.get(DigitalChannel.class,"allianceButton");
        if (allianceButton.getState()) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
            LED_red.on();
            LED_green.off();
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
            LED_red.off();
            LED_green.on();
        }
        blinkinLedDriver.setPattern(pattern);

        ///Motor Setup
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFront  = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");

        elevatorLift = myOpMode.hardwareMap.get(DcMotor.class, "elevatorLift");
        intakeSlide = myOpMode.hardwareMap.get(DcMotor.class,"intakeSlide");

        // Set Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        elevatorLift.setDirection(DcMotor.Direction.REVERSE);
        intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // elevatorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        intakePincher = myOpMode.hardwareMap.get(Servo.class, "intakePincher");
        intakePincher.setPosition(0);
        myOpMode.telemetry.addData("Intake Servo Pos",intakePincher.getPosition());
        //rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        //leftHand.setPosition(MID_SERVO);
        //rightHand.setPosition(MID_SERVO);


        //Limelight Setup
        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        //odo.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        //Telemetry Data
        myOpMode.telemetry.addData("Status", "Initialized");
        myOpMode.telemetry.addData("X offset", odo.getXOffset());
        myOpMode.telemetry.addData("Y offset", odo.getYOffset());
        myOpMode.telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        myOpMode.telemetry.addData("Device Scalar", odo.getYawScalar());
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param x     x-axis power
     * @param y     y-axis power
     * @param rotation Rotation
     */
    public void mecanumDrive(double x, double y, double rotation) {
        // Combine drive and turn for blended motion.
        double leftFrontPower = y + x + rotation;
        double rightFrontPower = y - x - rotation;
        double leftBackPower = y - x + rotation;
        double rightBackPower = y + x - rotation;

        // Normalize power values to keep them between -1 and 1
        double maxPower = Math.max(1.0, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightBackPower /= maxPower;

        // Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontPower Left Front Power
     * @param rightFrontPower Right Front Power
     * @param leftBackPower Left Back Power
     * @param rightBackPower Right Back Power
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        myOpMode.telemetry.addData("Front Left Power", leftFrontPower);
        myOpMode.telemetry.addData("Front Right Power", rightFrontPower);
        myOpMode.telemetry.addData("Rear Left Power", leftBackPower);
        myOpMode.telemetry.addData("Rear Right Power", rightBackPower);
        myOpMode.telemetry.update();
    }

    /**
     * Pass desired power to elevator
     *
     * @param elevatorPower Elevator Power
     */
    public void runElevator (double elevatorPower){

        elevatorLift.setPower(elevatorPower);
    }

    /**
     *
     */

    public void CloseIntakePincher (){
        intakePincher.setPosition(0);
    }
    public void OpenIntakePincher (){
        intakePincher.setPosition(0.60);
    }

    public void IntakePosition (double PosChange){
        double CurrentPosition = intakePincher.getPosition();
        double NewPosition = CurrentPosition + PosChange;
        intakePincher.setPosition(NewPosition);
        myOpMode.telemetry.addData("Intake Servo Pos", intakePincher.getPosition());
        myOpMode.telemetry.update();
    }


    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    //public void setHandPositions(double offset) {
    //    offset = Range.clip(offset, -0.5, 0.5);
    //    leftHand.setPosition(MID_SERVO + offset);
    //    rightHand.setPosition(MID_SERVO - offset);
    //}
}
