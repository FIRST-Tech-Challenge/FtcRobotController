package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

import java.util.List;

@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends BaseAutonomous {
    double motorPower;
    double x;
    double width = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize
        initialize();

        // activate TensorFlow on robot camera
        tfod.activate();
        tfod.setZoom(1.0, 16.0 / 9.0);

        // grab cone
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);

        // wait for start
        waitForStart();

        // raise slides to stow position
        driveSlidesAutonomous(Constants.SLIDE_STOW);

        // detect signal
        detectSignal();

        // drive forward 54 inches
        // todo - 54?
        driveInches(0, 54);

        // turn to -45 degrees
        turnToAngle(-45);

        // todo - 20 pixels?
        while (Math.abs(Constants.ROBOT_CAMERA_CENTER_X - x) > 20 && opModeIsActive()) {
            for (Recognition recognition : tfod.getRecognitions()) {
                if (recognition.getWidth() > width) {
                    x = (recognition.getLeft() + recognition.getRight()) * 0.5;
                    width = recognition.getWidth();
                }
            }

            // todo - 0.005?
            motorPower = (Constants.ROBOT_CAMERA_CENTER_X - x) * 0.005;

            motorFL.setPower(motorPower);
            motorFR.setPower(-motorPower);
            motorBL.setPower(motorPower);
            motorBR.setPower(-motorPower);
        }
    }
}
