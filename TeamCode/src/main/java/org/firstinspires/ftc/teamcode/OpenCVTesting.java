package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "HornetSquad: OpenCV Testing", group = "Hornet Vision")
public class OpenCVTesting extends LinearOpMode {
    private RobotHardware robot = new RobotHardware(this);
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    static final double FORWARD_SPEED = 0.2;
    static final double TURNSPEED = 0.2;
    @Override
    public void runOpMode() {
        robot.init();

        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam1"), visionProcessor);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            FirstVisionProcessor.Selected selectedDirection = visionProcessor.getSelection();

            telemetry.addData("Identified", selectedDirection);
            telemetry.update();

            if (selectedDirection == FirstVisionProcessor.Selected.LEFT)
                TravelLeft();
            else if (selectedDirection == FirstVisionProcessor.Selected.RIGHT)
                TravelRight();
            else
                //go straight
                TravelStraight();
            telemetry.update();
        }
    }

    private void TravelLeft()
    {
        telemetry.addData("Go left", "");
        //robot.driveRobot(FORWARD_SPEED, TURNSPEED);
    }
    private void TravelRight()
    {
        telemetry.addData("Go Right", "");
        //robot.driveRobot(FORWARD_SPEED, -TURNSPEED);
    }
    private void TravelStraight()
    {
        telemetry.addData("Go straight", "");
        //robot.driveRobot(FORWARD_SPEED, 0);
    }
}
