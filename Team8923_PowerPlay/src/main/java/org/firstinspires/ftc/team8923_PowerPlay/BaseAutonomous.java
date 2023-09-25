package org.firstinspires.ftc.team8923_PowerPlay;

import static org.firstinspires.ftc.team8923_PowerPlay.Constants.TICKS_PER_INCH;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// broken code: SDK 9.0 removed Vuforia
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team8923_PowerPlay.BaseOpMode;
import org.tensorflow.lite.support.label.TensorLabel;

import java.util.List;

public abstract class BaseAutonomous extends ConceptTensorFlowObjectDetectionWebcam {

    private ElapsedTime runtime = new ElapsedTime();

    double robotX;
    double robotY;
    double robotAngle;
    double headingOffset = 0.0;

    double speedFL;
    double speedFR;
    double speedBL;
    double speedBR;

    // Used to calculate distance traveled between loops
    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    // Variable for dectecion
    enum Position {
        ONE,
        TWO,
        THREE,
    }

    public void initAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initHardware();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Init State", "Init Finished");

        // Set last know encoder values
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        // set IMU heading offset
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //broken code:tfod.setZoom(1.0, 16.0/9.0);
        }

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    /**
     * Detects signal sleeve image.
     * @return the position of what the detection detects. That position is where we park.
     **/
    public Position detectSignalSleeve() {
        Position position = Position.THREE;
        // sets parking position dependant on label
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            sleep(10000);
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    if (recognition.getLabel().equals("Orange Leaves")) {
                        position = Position.ONE;
                    } else if (recognition.getLabel().equals("Blue Bats")) {
                        position =  Position.TWO;
                    } else if (recognition.getLabel().equals("Pink Flowers")){
                        position = Position.THREE;
                    }

                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                }
                telemetry.update();
            }
        }
        return position;
    }

    /**
     * Moves the robot forward, backwards, and strafes right and left in inches.
     * @param x strafes the robot right x inches. If moving left, pass in negative value.
     * @param y moves the robot forward y inches. If moving backward, pass in negative value.
     */
    public void driveInches(double x, double y) {
        double xTicks = x * TICKS_PER_INCH;
        double yTicks = y * TICKS_PER_INCH;

        double targetFL = xTicks + yTicks;
        double targetFR = yTicks - xTicks;
        double targetBL = yTicks - xTicks;
        double targetBR = yTicks + xTicks;

        // Determine new target position, and pass to motor controller
        targetFL += motorFL.getCurrentPosition();
        targetFR += motorFR.getCurrentPosition();
        targetBL += motorBL.getCurrentPosition();
        targetBR += motorBR.getCurrentPosition();

        motorFL.setTargetPosition((int) targetFL);
        motorFR.setTargetPosition((int) targetFR);
        motorBL.setTargetPosition((int) targetBL);
        motorBR.setTargetPosition((int) targetBR);

        // Turn On RUN_TO_POSITION
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        motorFL.setPower(0.5);
        motorFR.setPower(0.5);
        motorBL.setPower(0.5);
        motorBR.setPower(0.5);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < 30) &&
                (motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy())) {
        }

        // Stop all motion;
        stopDriving();

        // Turn off RUN_TO_POSITION
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // using imu
    public void imuPivot(double referenceAngle, double targetAngle, double maxSpeed, double kAngle, double timeout) {
        runtime.reset();
        // counter-clockwise is positive
        double pivot;
        double currentRobotAngle;
        double angleError;

        targetAngle = referenceAngle + targetAngle;
        targetAngle = adjustAngles(targetAngle);
        do {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            if (pivot >= 0.0) {
                pivot = Range.clip(pivot, 0.15, maxSpeed);
            } else {
                pivot = Range.clip(pivot, -maxSpeed, -0.15);
            }

            speedFL = pivot;
            speedFR = pivot;
            speedBL = pivot;
            speedBR = pivot;

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);
            idle();
        } while ((opModeIsActive() && (Math.abs(angleError) > 3.0)) && (runtime.seconds() < timeout));
        stopDriving();
    }

    /**
     * sets power to 0 to stop driving.
     */
    private void stopDriving() {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    /**
     * @param angle correction for irregular angles.
     * @return returns correct angle.
     */
    private double adjustAngles(double angle) {
        while (angle > 180)
            angle -= 360;
        while (angle < -180)
            angle += 360;
        return angle;
    }
}