package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;

@TeleOp(name = "Red Side NN Test", group = "Macro Tests")
@Disabled

public class NeuralNetworkLimelight3ARed extends LinearOpMode{

    private Limelight3A limelight;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    // Target class name to detect
    private final String TARGET_CLASS_NAME_BLUE = "blue-face-of-piece";
    private final String TARGET_CLASS_NAME_RED = "red-face-of-piece";
    private final String TARGET_CLASS_NAME_YELLOW = "yellow-face-of-piece";

    // to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect;

    @Override
    public void runOpMode() throws InterruptedException {
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                // rumble right motor 30% for 100 mSec
                .addStep(0.3, 0.3, 100)
                .build();

        // initialize Limelight and motors
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontL");
        backLeftMotor = hardwareMap.dcMotor.get("backL");
        frontRightMotor = hardwareMap.dcMotor.get("frontR");
        backRightMotor = hardwareMap.dcMotor.get("backR");

        // frequency that telemetry is sent to driver hub
        telemetry.setMsTransmissionInterval(5);

        // switch to neural network pipeline (assuming it's pipeline 1)
        limelight.pipelineSwitch(1);


        // starts looking for data, make sure to call start() or getLatestResult() will return null.

        // initializes the limelight
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // get Limelight status and update telemetry
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL Temp", "%.1fC", status.getTemp());
            telemetry.addData("LL CPU", "%.1f%%", status.getCpu());
            telemetry.addData("Pipeline", status.getPipelineIndex());

            // get the latest neural network result
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // access classifier results from the neural network
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();

                // check each classifier result for our target object
                boolean targetDetected = false;
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    telemetry.addData("Class", cr.getClassName());
                    telemetry.addData("Confidence", "%.2f", cr.getConfidence());

                    // if the target object is detected, stop the robot
                    if (cr.getConfidence() > 70) {
                        if (cr.getClassName().equals(TARGET_CLASS_NAME_RED) || cr.getClassName().equals(TARGET_CLASS_NAME_YELLOW)) {
                            targetDetected = true;
                            backLeftMotor.setPower(0);
                            frontLeftMotor.setPower(0);
                            frontRightMotor.setPower(0);
                            backRightMotor.setPower(0);
                            telemetry.addData("Status", "Target detected! Robot stopped. Rumblinnn");
                            gamepad1.runRumbleEffect(customRumbleEffect);
                            break;
                        }
                    }
                }

                // if the target is not detected, the robot continues to strafe left
                if (!targetDetected) {
                    frontLeftMotor.setPower(-0.5);   // Move backward
                    frontRightMotor.setPower(0.5);   // Move forward
                    backLeftMotor.setPower(0.5);     // Move forward
                    backRightMotor.setPower(-0.5);   // Move backward
                    telemetry.addData("Status", "Target not detected, strafing left...");
                }

            } else {
                telemetry.addData("Limelight", "No data available");
                frontLeftMotor.setPower(-0.5);   // Move backward
                frontRightMotor.setPower(0.5);   // Move forward
                backLeftMotor.setPower(0.5);     // Move forward
                backRightMotor.setPower(-0.5);   // Move backward
            }

            telemetry.update();
        }
        limelight.stop();
    }
}
