package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//CRServo
@Disabled
public class KryptoDragonsExampleCode extends LinearOpMode {
    private CRServo intakeServo;
    private DcMotor armMotor;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        //intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        //armMotor = hardwareMap.get(DcMotor.class,"armMotor");
        //frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        //frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        telemetry.addData("Hello",", Team RoboActive");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double initialMotorSpeed = 0.1;
        double currentMotorSpeed = 0.1;
        double maxReverseSpeed = -1.0;
        double maxForwardSpeed = 1.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //armMotor.setPower(0);
            //frontLeft.setPower(0.1);
            //frontRight.setPower(0.1);
            backLeft.setPower(0);
            backRight.setPower(0);

            if(gamepad1.y) {
                // Stop
                backLeft.setPower(0);
                backRight.setPower(0);
            } else if (gamepad1.dpad_up) {
                // Move forward
                backLeft.setPower(-currentMotorSpeed);
                backRight.setPower(currentMotorSpeed);
                telemetry.addData("Motor running forward at", currentMotorSpeed);
                telemetry.update();
            } else if (gamepad1.dpad_down) {
                // Move backward
                backLeft.setPower(currentMotorSpeed);
                backRight.setPower(-currentMotorSpeed);
                telemetry.addData("Motor running backward at", currentMotorSpeed);
                telemetry.update();
            } else if (gamepad1.dpad_left) {
                // Turn left
                backLeft.setPower(currentMotorSpeed);
                backRight.setPower(currentMotorSpeed);
                telemetry.addData("Motor running at", currentMotorSpeed);
                telemetry.update();
            } else if (gamepad1.dpad_right) {
                // Turn right
                backLeft.setPower(-currentMotorSpeed);
                backRight.setPower(-currentMotorSpeed);
                telemetry.addData("Motor running at", currentMotorSpeed);
                telemetry.update();
            } else if (gamepad1.right_stick_y == 1) {
                telemetry.addData("Hello","World");
                telemetry.update();
            } else if (gamepad1.right_bumper) {
                // Increase power by 0.1
                sleep(250);
                currentMotorSpeed = Range.clip(currentMotorSpeed + 0.1, 0.1, 1.0);
                telemetry.addData("Increment power to ", currentMotorSpeed);
                telemetry.update();
            } else if (gamepad1.left_bumper) {
                // Reduce power by 0.1
                sleep(100);
                currentMotorSpeed = Range.clip(currentMotorSpeed - 0.1, 0.1, 1.0);
                telemetry.addData("Reduce power to ", currentMotorSpeed);
                telemetry.update();
                waitForStart();





        }
    }
}}
