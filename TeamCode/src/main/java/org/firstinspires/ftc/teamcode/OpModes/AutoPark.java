
package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Park", group = "Concept")
public class AutoPark extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;
    public int parkingTarget = 2;
    Robot robot = new Robot();

    public enum AutoSteps {
        detectSignal, vSlider, deliverPreLoad, CycleThreeCones, park, endAuto
    }

    public AutoSteps Step = AutoSteps.detectSignal;

    @Override
    public void runOpMode() throws InterruptedException {

        // Tensorflow object detector requires initialization of hardware map and vuforia.
        // Following order is important.

        robot.init(hardwareMap);
        robot.initVuforia();
        robot.initTfod();



//        robot.vSlider.setTargetPosition(-165);
//        robot.vSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.vSlider.setPower(0.6);

        robot.claw.setPosition(1);
        robot.SwingArmToPosition(0.6,65);
        robot.swingArm.setPower(robot.swingArmHoldingPower);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("FL Motor Encoder", robot.FLMotor.getCurrentPosition());
        telemetry.addData("BL Motor Encoder", robot.BLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Encoder", robot.BRMotor.getCurrentPosition());
        telemetry.addData("FR Motor Encoder", robot.FRMotor.getCurrentPosition());
        telemetry.addData("VSlider Encoder ", robot.vSlider.getCurrentPosition());
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (robot.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());
                            String objectLabel = recognition.getLabel();
                            if (objectLabel == robot.LABELS[0]) {
                                parkingTarget = 1;
                            } else if (objectLabel == robot.LABELS[1]) {
                                parkingTarget = 2;
                            } else if (objectLabel == robot.LABELS[2]) {
                                parkingTarget = 3;
                            }
                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                            telemetry.addData("Robot Location", robot.Location);
                            telemetry.addData("Vslider Encoder", robot.vSlider.getCurrentPosition());
                        }
                        telemetry.update();
                    }
                }

                switch (Step) {
                    case detectSignal:
                        telemetry.addData("Parking Target ", parkingTarget);
                        telemetry.update();
                        Step = AutoSteps.park;
                        break;

//                    case vSlider:
//                        robot.MoveSlider(0.8,1000);
//                        Step = AutoSteps.endAuto;
//                        break;
//
//                    case deliverPreLoad:
//                        DeliverPreLoad();
//                        Step = AutoSteps.park;
//                        break;
//
//                    case CycleThreeCones:
//                        //Drive to the stack
//                        robot.DriveToPosition(0.8, -35, 60, false);
//                        robot.turnRobotToAngle(90);
//                        for(int i = 0; i < 4; i++) {
//                            CycleCone();
//                        }
//                        Step = AutoSteps.park;
//                        break;

                    case park:
                        Park(parkingTarget);
                        Step = AutoSteps.endAuto;
                        break;

                    case endAuto:
                        telemetry.addData("➡️", "Auto Finished");
                        telemetry.update();
                        break;
                }
            }
        }
    }



    public void Park(int location) {
        if (location == 1) {
            robot.claw.setPosition(0);
            robot.DriveToPosition(0.3, 75, 70, true);
        }

        if (location == 2) {
            robot.claw.setPosition(0);
            robot.DriveToPosition(0.3, 0, 70, true);
        }

        if (location == 3) {
            robot.claw.setPosition(0);
            robot.DriveToPosition(0.3, -75, 70, true);

        }
    }
}
