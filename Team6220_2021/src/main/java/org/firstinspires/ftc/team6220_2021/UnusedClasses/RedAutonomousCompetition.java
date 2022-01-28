package org.firstinspires.ftc.team6220_2021.UnusedClasses;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team6220_2021.MasterAutonomous;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

import java.util.List;

@Disabled
@Autonomous(name = "Red Autonomous Competition", group = "Competition")
public class RedAutonomousCompetition extends MasterAutonomous {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();
        initVuforia();
        initTfod();

        int barcode = 2;

        servoGrabber.setPosition(Constants.CLOSED_GRABBER_POSITION);
        pauseMillis(1500);
        motorArm.setPower(0.5);
        motorArm.setTargetPosition(800);
        pauseMillis(750);
        servoArm.setPosition(0.667);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 2000) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel().equals("TSE")) {
                        double TSELocation = (recognition.getLeft() + recognition.getRight()) / 2.0;

                        if (TSELocation > Constants.TSE_START && TSELocation <= Constants.TSE_CENTER1) {
                            barcode = 0;
                        } else if (TSELocation > Constants.TSE_CENTER1 && TSELocation <= Constants.TSE_CENTER2) {
                            barcode = 1;
                        } else if (TSELocation > Constants.TSE_CENTER2 && TSELocation <= Constants.TSE_END) {
                            barcode = 2;
                        }
                    }
                }
                telemetry.update();
            }
        }

        waitForStart();

        if (tfod != null) {
            tfod.shutdown();
        }

        telemetry.addData("barcode: ", barcode);
        telemetry.update();

        driveInches(6, Constants.MIN_DRIVE_PWR, true);
        pauseMillis(125);
        turnToAngle(90);
        pauseMillis(125);
        driveInches(16, Constants.MIN_DRIVE_PWR, true);
        pauseMillis(125);
        motorLeftDuck.setPower(-0.4);
        pauseMillis(2000);
        motorLeftDuck.setPower(0.0);
        driveInches(4, Constants.MIN_DRIVE_PWR, false);
        pauseMillis(125);
        turnToAngle(10);
        pauseMillis(125);
        driveInches(36, Constants.MIN_DRIVE_PWR, true);
        pauseMillis(125);
        turnToAngle(-80);
        pauseMillis(125);

        switch (barcode) {
            case 0:
                motorArm.setTargetPosition(300);
                pauseMillis(750);
                servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 2400.0);
                driveInches(10, Constants.MIN_DRIVE_PWR, true);
                break;

            case 1:
                motorArm.setTargetPosition(550);
                pauseMillis(750);
                servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 2400.0);
                driveInches(12, Constants.MIN_DRIVE_PWR, true);
                break;

            case 2:
                motorArm.setTargetPosition(800);
                pauseMillis(750);
                servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION - motorArm.getCurrentPosition() / 2400.0);
                driveInches(14, Constants.MIN_DRIVE_PWR, true);
                break;
        }

        pauseMillis(500);
        servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);
        pauseMillis(500);

        switch (barcode) {
            case 0:
                driveInches(6, Constants.MIN_DRIVE_PWR, false);
                break;

            case 1:
                driveInches(8, Constants.MIN_DRIVE_PWR, false);
                break;

            case 2:
                driveInches(10, Constants.MIN_DRIVE_PWR, false);
                break;
        }

        pauseMillis(125);
        turnToAngle(170);
        pauseMillis(125);
        driveInches(6, Constants.MIN_DRIVE_PWR, true);

        motorBelt.setPower(0.75);
        motorBelt.setTargetPosition(1850);
        pauseMillis(750);
        servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
        motorArm.setTargetPosition(0);
    }
}