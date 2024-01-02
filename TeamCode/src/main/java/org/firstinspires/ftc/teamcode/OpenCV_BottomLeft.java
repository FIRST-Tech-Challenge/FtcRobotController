package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "HornetSquad: OpenCV Detection Bottom Left", group = "Hornet Vision")
public class OpenCV_BottomLeft extends LinearOpMode {
    private RobotHardware robot = new RobotHardware(this);
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    static final double FORWARD_SPEED = 0.2;
    static final double TURNSPEED = 0.2;
    @Override
    public void runOpMode() {
        robot.init();

        visionProcessor = new FirstVisionProcessor();
        visionProcessor.colorToCheck = "blue";
        //visionProcessor.telemetry = telemetry;

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam1"), visionProcessor);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //while (opModeIsActive() && !isStopRequested()) {
            robot.driveRobot(0.2, 0);
            sleep(10);
            robot.driveRobot(0, 0);
            FirstVisionProcessor.Selected centerSelectedDirection = visionProcessor.getSelection();
            double[] centerColorValues = visionProcessor.colorValues;
            telemetry.addData("CenterSelectionIdentified", centerSelectedDirection);
            telemetry.update();
            sleep(1000);
            robot.driveRobot(0, 0.2);
            sleep(50);
            FirstVisionProcessor.Selected leftSelectedDirection = visionProcessor.getSelection();
            double[] leftColorValues = visionProcessor.colorValues;
            telemetry.addData("LeftSelectionIdentified", leftSelectedDirection);
            telemetry.update();
            sleep(1000);
            robot.driveRobot(0, -0.4);
            sleep(50);
            FirstVisionProcessor.Selected rightSelectedDirection = visionProcessor.getSelection();
            double[] rightColorValues = visionProcessor.colorValues;
            telemetry.addData("RightSelectionIdentified", rightSelectedDirection);
            telemetry.update();

            FirstVisionProcessor.Selected selectedDirection = findDirection(leftSelectedDirection, leftColorValues,
                    centerSelectedDirection, centerColorValues, rightSelectedDirection, rightColorValues);

            if (selectedDirection == FirstVisionProcessor.Selected.LEFT)
                TravelLeft();
            else if (selectedDirection == FirstVisionProcessor.Selected.RIGHT)
                TravelRight();
            else
                //go straight
                TravelStraight();
            telemetry.addData("final ", selectedDirection);
            telemetry.update();

            robot.driveRobot(0,0);
            sleep(5000);



        //}
    }

    private FirstVisionProcessor.Selected findDirection(FirstVisionProcessor.Selected leftSelected, double[] leftColorValues,
                                                        FirstVisionProcessor.Selected middleSelected, double[] middleColorValues,
                                                        FirstVisionProcessor.Selected rightSelected, double[] rightColorValues)
    {
        if (leftSelected == FirstVisionProcessor.Selected.RIGHT && middleSelected == FirstVisionProcessor.Selected.MIDDLE && rightSelected == FirstVisionProcessor.Selected.LEFT)
            return FirstVisionProcessor.Selected.MIDDLE;
        else if (leftSelected == FirstVisionProcessor.Selected.LEFT && rightSelected == FirstVisionProcessor.Selected.RIGHT && leftColorValues[0] > rightColorValues[2])
            return FirstVisionProcessor.Selected.LEFT;
        else if (leftSelected == FirstVisionProcessor.Selected.LEFT && rightSelected == FirstVisionProcessor.Selected.RIGHT && leftColorValues[0] < rightColorValues[2])
            return FirstVisionProcessor.Selected.RIGHT;
        return FirstVisionProcessor.Selected.MIDDLE;

    }
    private void TravelLeft()
    {
        telemetry.addData("Go left", "");
        //robot.driveRobot(FORWARD_SPEED, TURNSPEED);
    }
    private void TravelRight()
    {
        telemetry.addData("Go Right", "");
         //From Samarth -commenting as its not tested
        //near the bar
        //move forward while turning to right

        /*
        robot.driveRobot(-FORWARD_SPEED, TURNSPEED);
        sleep(3500);
        robot.driveRobot(0,0);
        robot.driveRobot(FORWARD_SPEED, 0);
        sleep(2000);
        robot.moveArmFullSpeed(RobotHardware.ARM_DOWN_POWER);
        sleep(2700);
        robot.stopArm();

        robot.driveRobot(FORWARD_SPEED, 0);
        sleep(4200);

        robot.driveRobot(0,TURNSPEED);
        sleep(5200);

        robot.driveRobot(FORWARD_SPEED,0);
        sleep(8000);

        robot.moveElbowToPosition(0.3);
        sleep(1000);

        robot.moveGrabberToPosition(RobotHardware.GRABBER_MIN);
        sleep(500);

        robot.moveElbowToPosition(-0.3);
        sleep(1000);

        robot.driveRobot(FORWARD_SPEED,0);
        sleep(1000);

         */
    }
    private void TravelStraight()
    {

    }
}
