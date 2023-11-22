package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;


    OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);

    public void initOpenCv() {
        // initialize the webcam and openCV pipeline
        myColorDetection.init();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        myColorDetection.detectColor();

        while (opModeIsActive()) {
            // print detection status and coordinates of largest object
            telemetry.addLine("Detection")
                    .addData(" ", myColorDetection.targetDetected)
                    .addData("x", myColorDetection.targetPoint.x)
                    .addData("y", myColorDetection.targetPoint.y);

            telemetry.addData("Frame Count", myColorDetection.robotCamera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", myColorDetection.robotCamera.getFps()));
            telemetry.addData("Total frame time ms", myColorDetection.robotCamera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", myColorDetection.robotCamera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", myColorDetection.robotCamera.getOverheadTimeMs());
            telemetry.update();

            myColorDetection.robotCamera.stopStreaming();
            myColorDetection.robotCamera.closeCameraDevice();
        }
    }

    public void initAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initHardware();

        telemetry.addData("Init State", "Init Finished");

        // Set last know encoder values
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

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

    private void stopDriving() {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }
}
