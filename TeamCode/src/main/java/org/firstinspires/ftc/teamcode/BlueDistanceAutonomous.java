package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "blue-far-auto")
public class BlueDistanceAutonomous extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor linearSlide = null;
    private CRServo intake = null;
    double integralSum = 0;
    double Kp = 0.1;
    double Ki = 0.0;
    double Kd = 0.0;
    ElapsedTime timer = new ElapsedTime();

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */

    /**
     * The variable to store our instance of the vision portal.
     */

    @Autonomous(name="blue-far-auto")
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        linearSlide = hardwareMap.get(DcMotor.class, "outputslide");
        intake = hardwareMap.crservo.get("outputservo");

        //  leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //  leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //  rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        //  rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // turn left
                // PIDControl(leftFrontDrive, 100, leftFrontDrive.getCurrentPosition());
                // PIDControl(rightFrontDrive, 100, rightFrontDrive.getCurrentPosition());
                // PIDControl(leftBackDrive, 100, leftBackDrive.getCurrentPosition());
                // PIDControl(rightBackDrive, 100, rightBackDrive.getCurrentPosition());
                //moveToPosition(-1500);
                //sleep(4000);
                //linearSlide.setPower(0.1);
                moveForwardToThree();
                resetPower();
                //turnLeft();
                strafeLeft();
                //resetPower();
                //             telemetryTfod();
                //             if(telemetryTfod() > 0) {
                //                 telemetry.addData("???????????", 0);
                //                 // pick up pixel
                //             }else {
                //                 //move backwards
                //                  PIDControl(leftFrontDrive, -100, leftFrontDrive.getCurrentPosition());
                //                 PIDControl(rightFrontDrive, -100, rightFrontDrive.getCurrentPosition());
                //                 PIDControl(leftBackDrive, -100, leftFrontDrive.getCurrentPosition());
                //                 PIDControl(rightBackDrive, -100, leftFrontDrive.getCurrentPosition());
                //                 //turn right
                //                 PIDControl(leftFrontDrive, 100, leftFrontDrive.getCurrentPosition());
                //                 PIDControl(rightFrontDrive, -100, rightFrontDrive.getCurrentPosition());
                //                 PIDControl(leftBackDrive, 100, leftFrontDrive.getCurrentPosition());
                //                 PIDControl(rightBackDrive, -100, leftFrontDrive.getCurrentPosition());
                //                 //move forward
                //                 PIDControl(leftFrontDrive, 100, leftFrontDrive.getCurrentPosition());
                //                 PIDControl(rightFrontDrive, 100, rightFrontDrive.getCurrentPosition());
                //                 PIDControl(leftBackDrive, 100, leftFrontDrive.getCurrentPosition());
                //                 PIDControl(rightBackDrive, 100, leftFrontDrive.getCurrentPosition());
                //                 //check for pixel
                //                 telemetryTfod();
                //                 if(telemetryTfod() > 0) {
                //                     telemetry.addData("???????????", 0);
                //                     // pick up pixel
                //                 }else {
                //                 //move backwards
                //                     PIDControl(leftFrontDrive, -100, leftFrontDrive.getCurrentPosition());
                //                     PIDControl(rightFrontDrive, -100, rightFrontDrive.getCurrentPosition());
                //                     PIDControl(leftBackDrive, -100, leftFrontDrive.getCurrentPosition());
                //                     PIDControl(rightBackDrive, -100, leftFrontDrive.getCurrentPosition());
                //                     //turn right
                //                     PIDControl(leftFrontDrive, 100, leftFrontDrive.getCurrentPosition());
                //                     PIDControl(rightFrontDrive, -100, rightFrontDrive.getCurrentPosition());
                //                     PIDControl(leftBackDrive, 100, leftFrontDrive.getCurrentPosition());
                //                     PIDControl(rightBackDrive, -100, leftFrontDrive.getCurrentPosition());
                //                     //move forward
                //                     PIDControl(leftFrontDrive, 100, leftFrontDrive.getCurrentPosition());
                //                     PIDControl(rightFrontDrive, 100, rightFrontDrive.getCurrentPosition());
                //                     PIDControl(leftBackDrive, 100, leftFrontDrive.getCurrentPosition());
                //                     PIDControl(rightBackDrive, 100, leftFrontDrive.getCurrentPosition());
                //                     //check for pixel
                //                     telemetryTfod();
                //                     if(telemetryTfod() > 0) {
                //                         telemetry.addData("???????????", 0);
                //                         // pick up pixel
                //                     }
                //                     //turn left
                //                     PIDControl(leftFrontDrive, -100, leftFrontDrive.getCurrentPosition());
                //                     PIDControl(rightFrontDrive, 100, rightFrontDrive.getCurrentPosition());
                //                     PIDControl(leftBackDrive, -100, leftFrontDrive.getCurrentPosition());
                //                     PIDControl(rightBackDrive, 100, leftFrontDrive.getCurrentPosition());
                //                 }
                //             }
                //             // moving to the board:
                //             // turn left
                //             PIDControl(leftFrontDrive, -100, leftFrontDrive.getCurrentPosition());
                //             PIDControl(rightFrontDrive, 100, rightFrontDrive.getCurrentPosition());
                //             PIDControl(leftBackDrive, -100, leftFrontDrive.getCurrentPosition());
                //             PIDControl(rightBackDrive, 100, leftFrontDrive.getCurrentPosition());
                //             //go forward
                //             PIDControl(leftFrontDrive, 500, leftFrontDrive.getCurrentPosition());
                //             PIDControl(rightFrontDrive, 500, rightFrontDrive.getCurrentPosition());
                //             PIDControl(leftBackDrive, 500, leftFrontDrive.getCurrentPosition());
                //             PIDControl(rightBackDrive, 500, leftFrontDrive.getCurrentPosition());
                //             //raise linear slide
                //             PIDControl(linearSlide, 500, linearSlide.getCurrentPosition());
                //             //empty pixel on to board
                //             //TODO: FIND OUT HOW TO USE PID ON A SERVO
                //             // Push telemetry to the Driver Station.
                //             telemetry.update();

                //             // Save CPU resources; can resume streaming when needed.
                //             // Share the CPU.
                //             sleep(20);
                //         }
                //     }

                // }   // end runOpMode()

                // /**
                //  * Initialize the TensorFlow Object Detection processor.
                //  */

                // /**
                // *// Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
                // */
                telemetry.addData("leftfront", leftFrontDrive.getCurrentPosition());
                telemetry.addData("rightfront", rightFrontDrive.getCurrentPosition());
                telemetry.addData("leftback", leftBackDrive.getCurrentPosition());
                telemetry.addData("rightback", rightBackDrive.getCurrentPosition());
                telemetry.addData("linearslide", linearSlide.getCurrentPosition());
                telemetry.update();
            }}}
    public void moveToPosition(double reference, DcMotor motor) {
        while (motor.getCurrentPosition() != reference) {
            double power = PIDControl(reference, reference, motor);
            motor.setPower(power);
        }
    }
    public double PIDControl(double reference, double lastError, DcMotor motor) {
        double state = motor.getCurrentPosition();
        double error = reference - state;
        if(error < 100 && error > -100) {
            error = 0;
        }
        integralSum += error * timer.seconds();
        double derivative = (error-lastError) / timer.seconds();

        lastError = error;

        timer.reset();

        double out = (error*Kp) + (derivative * Kd) + (integralSum * Ki);
        telemetry.addData("out", out);
        telemetry.addData("position", state);
        telemetry.update();
        return out;
    }
    public void moveForwardToThree() {
        int variance = -100;
        int desiredLoc = -1700;
        while(leftFrontDrive.getCurrentPosition() > -1600
                || leftFrontDrive.getCurrentPosition() < -1710) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(power * 0.5);
            rightFrontDrive.setPower(-power * 0.5);
            leftBackDrive.setPower(power * 0.5);
            rightBackDrive.setPower(-power * 0.5);
        }
    }

    public void resetPower() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void strafeLeft() {
        int desiredLoc = -1500;
        int variance = -25;
        while(leftFrontDrive.getCurrentPosition() > desiredLoc - variance
                || leftFrontDrive.getCurrentPosition() < desiredLoc + variance
        ) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(-power);
        }
    }

    public void turnLeft() {
        int variance = 25;
        int desiredLoc = -1350;
        while(leftFrontDrive.getCurrentPosition() < desiredLoc - variance
                || leftFrontDrive.getCurrentPosition() > desiredLoc + variance) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        }
    }
}