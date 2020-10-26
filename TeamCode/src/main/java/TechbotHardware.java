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

import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

/*
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class TechbotHardware {
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    //public DcMotor farm = null;

    // Touch sensors
    public TouchSensor touchSensor1 = null;
    public TouchSensor touchSensor2 = null;

    //camera
    public CameraName cameraName = null;

    //public Servo servoHand = null;
    //public Servo fervoL = null;
    //public Servo fervoR = null;
    public Servo wobbleClamp = null;
    public Servo wobbleLift = null;

    public ElapsedTime runtime;
    //public Servo servoWrist = null;

    public static final double MID_SERVO = 0.5;
    //public static final double ARM_UP_POWER = 0.45;
    //public static final double ARM_DOWN_POWER = -0.45;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TechbotHardware() {

    }

    public void slideL(double slideDrive) {
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(slideDrive);
        rightDrive.setPower(slideDrive);
        leftBackDrive.setPower(slideDrive);
        rightBackDrive.setPower(slideDrive);
    }

    public void slideR(double slideDrive) {
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(slideDrive);
        rightDrive.setPower(slideDrive);
        leftBackDrive.setPower(slideDrive);
        rightBackDrive.setPower(slideDrive);
    }

    //NO TELEOP
    public void slideByTime (double slideDrive, double slideTime) {
        this.slideL(slideDrive);
        while (runtime.seconds() < slideTime) {
            // do nothing
        }
    }

    public void slideS(double slideSDrive) {
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(slideSDrive);
        rightDrive.setPower(slideSDrive);
        leftBackDrive.setPower(slideSDrive);
        rightBackDrive.setPower(slideSDrive);

    }

    //NO TELEOP
    public void slidebytime(double slideSDrive, double slideTime) {
        this.slideL(slideSDrive);
        while (runtime.seconds() < slideTime);
        //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
        //telemetry.update();
    }

    public void drive(double driveDrive) {
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(driveDrive);
        rightDrive.setPower(driveDrive);
        leftBackDrive.setPower(driveDrive);
        rightBackDrive.setPower(driveDrive);
    }

    //NO TELEOP
    public void drivebytime(double driveDrive, double driveTime) {
        this.drive(driveDrive);
        while (runtime.seconds() < driveTime);
        //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
        //telemetry.update();
    }

    public void driveS(double driveSDrive) {
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(driveSDrive/2);
        rightDrive.setPower(driveSDrive/2);
        leftBackDrive.setPower(driveSDrive/2);
        rightBackDrive.setPower(driveSDrive/2);

    }

    public void spin(double spinDrive) {
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(spinDrive);
        rightDrive.setPower(spinDrive);
        leftBackDrive.setPower(spinDrive);
        rightBackDrive.setPower(spinDrive);
    }

    /*public void drag(double dragDrive){
        fervoL.setPosition(MID_SERVO);
        fervoR.setPosition(MID_SERVO);

        fervoL.setPosition(dragDrive);
        fervoR.setPosition(dragDrive);
    }
    */

    public void stop() {
        this.drive(0);
    }
      /*  double power = 0.0;

        leftDrive.setPower(power);
        rightDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
    */


    public void chill(double chillTime) {
        this.stop();
        while (runtime.seconds() < chillTime) {
            // do nothing
        }
    }

    public void driveByTime(double drive, double driveTime) {
        this.drive(drive);
        while (runtime.seconds() < driveTime) {
            // do nothing
        }
    }

    public void spinByTime(double spinDrive, double spinTime) {
        this.spin(spinDrive);
        while (runtime.seconds() < spinDrive) {
            // do nothing
        }
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        runtime = new ElapsedTime();

        leftDrive  = hwMap.get(DcMotor.class, "Left front motor");
        rightDrive = hwMap.get(DcMotor.class, "Right front motor");
        leftBackDrive = hwMap.get(DcMotor.class, "Left back motor");
        rightBackDrive = hwMap.get(DcMotor.class, "Right back motor");
        //farm = hwMap.get(DcMotor.class, "foundation motor");
        //servoHand = hwMap.get(Servo.class, "hand");
        touchSensor1 = hwMap.touchSensor.get("touchSensor1");
        touchSensor2 = hwMap.touchSensor.get("touchSensor2");
        wobbleClamp = hwMap.get(Servo.class, "wobbleClamp");
        wobbleLift = hwMap.get(Servo.class,"wobbleLift");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        cameraName = hwMap.get(CameraName.class, "camera");
        //farm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
     /*   leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        farm.setPower(0);*/


        /*
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
        // Define and initialize ALL installed servos.
        //servoHand  = hwMap.get(Servo.class, "hand");
        //servoWrist = hwMap.get(Servo.class, "wrist");
        // servoHand.setPosition(0);
        // fervoL = hwMap.get(Servo.class, "fervoL");
        // fervoR = hwMap.get(Servo.class, "fervoR");
        // fervoL.setPosition(MID_SERVO);
        // fervoR.setPosition(MID_SERVO);
        //servoWrist.setPosition(MID_SERVO);
        wobbleClamp.setPosition(MID_SERVO);
        wobbleLift.setPosition(MID_SERVO);
    }
}

