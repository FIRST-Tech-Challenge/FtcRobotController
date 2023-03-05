

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Right", group = "Auto")
public class AutoRight extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;
    public int parkingTarget = 2;
    Robot robot = new Robot();

    public enum AutoSteps {
        detectSignal, deliverPreLoad, cycleCones, parkFromMedium, endAuto
    }

    public AutoSteps Step = AutoSteps.detectSignal;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.init(hardwareMap);
        robot.initVuforia();
        robot.initTfod();
        robot.initArmClaw();


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (robot.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            String objectLabel = recognition.getLabel();
                            if (objectLabel == robot.LABELS[0]) {
                                parkingTarget = 1;
                            } else if (objectLabel == robot.LABELS[1]) {
                                parkingTarget = 2;
                            } else if (objectLabel == robot.LABELS[2]) {
                                parkingTarget = 3;
                            }
//
//                            telemetry.addData("", " ");
//                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                            telemetry.addData("Robot Location", robot.Location);
                            telemetry.addData("parking target", parkingTarget);
                        }
                        telemetry.update();
                    }
                }

                switch (Step) {
                    case detectSignal:
                        telemetry.addData("Parking Target ", parkingTarget);
                        telemetry.update();
                        Step = AutoSteps.deliverPreLoad;
                        break;

                    case deliverPreLoad:
                        robot.deliverPreLoad(false);
                        Step = AutoSteps.parkFromMedium;
                        break;

                    case parkFromMedium:
                        robot.ParkFromMedium(parkingTarget, true);
                        Step = AutoSteps.endAuto;

                    case endAuto:
                        telemetry.addData("➡️", "Auto Finished");
                        telemetry.update();
                        break;
                }
            }
        }
    }
}