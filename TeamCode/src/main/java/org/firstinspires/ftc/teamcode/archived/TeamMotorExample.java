package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//CRServo
@Disabled
public class TeamMotorExample extends LinearOpMode {
    private DcMotor leftWheelMotor;
    private DcMotor rightWheelMotor;

    @Override
    public void runOpMode() {
        leftWheelMotor = hardwareMap.get(DcMotor.class, "leftWheelMotor");
        rightWheelMotor = hardwareMap.get(DcMotor.class, "rightWheelMotor");

        double tgtPower = 0;

        telemetry.addData("Hello", ", Team Krypto Dragons");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double position = 0.0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Status", "Game Started...");
            telemetry.update();

            if (gamepad1.b) {
                //stops the motor
                leftWheelMotor.setPower(0);
                rightWheelMotor.setPower(0);
                leftWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (gamepad1.dpad_up) {
                leftWheelMotor.setDirection(DcMotor.Direction.FORWARD);
                leftWheelMotor.setTargetPosition(1);
                leftWheelMotor.setPower(0.1);

                rightWheelMotor.setDirection(DcMotor.Direction.FORWARD);
                rightWheelMotor.setTargetPosition(1);
                rightWheelMotor.setPower(0.1);

                telemetry.addData("Dpad Up", "expect forward");
                telemetry.update();
            } else if (gamepad1.dpad_down) {
                leftWheelMotor.setDirection(DcMotor.Direction.REVERSE);
                leftWheelMotor.setTargetPosition(1);
                leftWheelMotor.setPower(0.1);

                rightWheelMotor.setDirection(DcMotor.Direction.REVERSE);
                rightWheelMotor.setTargetPosition(1);
                rightWheelMotor.setPower(0.1);

                telemetry.addData("Dpad Up", "expect forward");
                telemetry.update();
            } else if (this.gamepad1.right_bumper) {
                double existing_power = rightWheelMotor.getPower();
                rightWheelMotor.setPower(existing_power + 0.1);
                telemetry.addData("RightWheelMotor", "increase by 0.1");
                telemetry.update();
            } else if (this.gamepad1.left_bumper) {
                double existing_power = rightWheelMotor.getPower();
                rightWheelMotor.setPower(existing_power - 0.1);
                telemetry.addData("RightWheelMotor", "reduce by 0.1");
                telemetry.update();
            } else if (gamepad1.dpad_right) {
                leftWheelMotor.setDirection(DcMotor.Direction.FORWARD);
                leftWheelMotor.setTargetPosition(1);
                leftWheelMotor.setPower(0.1);

                rightWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                rightWheelMotor.setTargetPosition(1);
                rightWheelMotor.setPower(0.5);
            }else if (gamepad1.dpad_left){
                leftWheelMotor.setDirection(DcMotor.Direction.FORWARD);
                leftWheelMotor.setTargetPosition(1);
                leftWheelMotor.setPower(0.5);

                rightWheelMotor.setDirection(DcMotor.Direction.FORWARD);
                rightWheelMotor.setTargetPosition(1);
                rightWheelMotor.setPower(0.1);
                waitForStart();
            }


                telemetry.addData("left wheel motor position", leftWheelMotor.getTargetPosition());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
        }
    }
