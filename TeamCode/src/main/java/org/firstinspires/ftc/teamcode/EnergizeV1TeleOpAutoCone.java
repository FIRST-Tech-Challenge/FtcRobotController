package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Energize V1 TeleOp", group="Linear Opmode")
//@Disabled
public class EnergizeV1TeleOpAutoCone extends LinearOpMode {

    // Declares OpMode members for needed motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor1 = null;
   // private DcMotor liftMotor2 = null;
    private ElapsedTime     timer = new ElapsedTime();
    double liftPower = 0.0;
    double INCREMENT = 0.01;
    double NEGATIVE_INCREMENT = -0.2;
    double MAX_SPEED = 0.7;
    double MIN_SPEED = -0.7;
    //private DcMotor lift = null;
    //Servo rightServo;
   // Servo leftServo;
    double servoPosition = 0.0;

    @Override
    public void runOpMode() {

        // Corresponds driving motor names to motor variables.
        /*leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Corresponds driving servo names to servo variables and set beginning position.
       /* rightServo = hardwareMap.get(Servo.class,"rightservo");
        rightServo.setPosition(servoPosition);
        leftServo = hardwareMap.get(Servo.class,"leftservo");
        leftServo.setPosition(1.0);*/

        // Corresponds lift motor names to motor variables.
        liftMotor1  = hardwareMap.get(DcMotor.class, "liftMotor");
       // liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");

        // Initializes motor directions.
       /* leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);*/
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        //liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Waits for driver to press play.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Runs until driver presses stop.
        while (opModeIsActive()) {
            dualLift(0.0);
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combines the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalizes the values so no wheel power exceeds 100%.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Drops lift slightly
            if (gamepad2.right_trigger> 0.0 && liftPower >= MIN_SPEED) {
                for(int i = 0; i<8; i++){
                    liftPower -= INCREMENT;
                    dualLift(liftPower);
                    sleep(500);
                }
                //stop lift
                liftPower = 0.0;
                dualLift(liftPower);

                }
            if(gamepad2.left_trigger > 0.0 && liftPower >= MIN_SPEED) {
                // Opens claw
               // rightServo.setPosition(1.0);
               // leftServo.setPosition(0.0);
                sleep(1000);

                // Drops lift down (may make it so it goes down completely and stops when it hits touch sensor)
                for(int i = 0; i<8; i++) {
                    liftPower -= INCREMENT;
                    dualLift(liftPower);
                    sleep(2000);
                }
                // stop lift
                liftPower = 0.0;
                dualLift(liftPower);
            }







            // Closes claw.
            if (gamepad2.left_bumper) {
               // rightServo.setPosition(0.0);
               // leftServo.setPosition(1.0);
            }

            // Go up
            if (gamepad2.y && liftPower <= MAX_SPEED) {
                liftPower += INCREMENT;
            }

            // Go down
            else if (gamepad2.a && liftPower >= MIN_SPEED) {
                liftPower -= INCREMENT;
            }

            else if (gamepad2.b) {
                liftPower = 0.0;
            }

            // Kick starts lift motor.
            else if (gamepad2.x) {
                liftPower = -0.2;
                dualLift(liftPower);
                sleep(500);
                liftPower = 0.2;
            }

            // Sends calculated power to motors.
           /* leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);*/
            dualLift(liftPower);

            // Shows the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift Motors", "%5.2f", liftPower);
            telemetry.update();
        }

    }
    public void dualLift(double power) {
        liftMotor1.setPower(power);
       //liftMotor2.setPower(power);
    }

    }
