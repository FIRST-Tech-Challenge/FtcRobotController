package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.podservos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

@TeleOp(name = "Odometry Test")
@Disabled
public class OdometryTest extends GenericOpModeTemplate {


    @Override
    public void opModeMain() throws InterruptedException {
        this.initOdometryServos();
        podServos.raise();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        telemetry.addData("Wiggling Left Servo", 'y');
        telemetry.update();
        podServos.leftServo.setPosition(OdometryPodServos.leftServoLowerPosition);
        Thread.sleep(500);
        podServos.leftServo.setPosition(OdometryPodServos.leftServoRaisePosition);
        Thread.sleep(500);

        telemetry.addData("Wiggling Right Servo", 'y');
        telemetry.update();
        podServos.rightServo.setPosition(OdometryPodServos.rightServoLowerPosition);
        Thread.sleep(500);
        podServos.rightServo.setPosition(OdometryPodServos.rightServoRaisePosition);
        Thread.sleep(500);

        telemetry.addData("Wiggling Horizontal Servo", 'y');
        telemetry.update();
        podServos.horizontalServo.setPosition(OdometryPodServos.horizontalServoLowerPosition);
        Thread.sleep(500);
        podServos.horizontalServo.setPosition(OdometryPodServos.horizontalServoRaisePosition);
        Thread.sleep(500);

        while (opModeIsActive() && !isStopRequested()) {
            podServos.raise();
            Thread.sleep(5000);
            podServos.lower();
            Thread.sleep(5000);
        }

    }
}
