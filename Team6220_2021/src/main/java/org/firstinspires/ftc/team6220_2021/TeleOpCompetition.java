package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

@TeleOp(name = "TeleOp Competition", group = "Competition")
public class TeleOpCompetition extends MasterTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();

        servoArm.setPosition(Constants.SERVO_ARM_RESET_POSITION);
        servoGrabber.setPosition(Constants.OPEN_GRABBER_POSITION);

//        motorBelt.setPower(0.4);
//        motorBelt.setTargetPosition(Constants.BELT_RESET);

        motorArm.setPower(0.5);
        motorArm.setTargetPosition(-50);
        pauseMillis(500);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setPower(0.5);
        motorArm.setTargetPosition(Constants.ARM_COLLECTING_LEVEL);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoArm.setPosition(0.45 - 3 * motorArm.getCurrentPosition() / 8000.0);

        waitForStart();

        while (opModeIsActive()) {
            driver1.update();
            driver2.update();

            driveRobot();
            driveLeftCarousel();
            driveRightCarousel();
            driveGrabber();
            driveArm();
            driveArmManual();
            reset();
        }
    }
}