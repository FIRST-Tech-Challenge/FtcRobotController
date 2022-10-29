package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="DriveOfficial")
public class MecanumTeleOp extends LinearOpMode {
    private final double inches_per_revolution = 60/25.4*Math.PI; //60 mm * (1 inches)/(25.4 mm) is the diameter of the wheel in inches, *pi for circumference
    private final double ticks_per_revolution = 360*6.0; //4 ticks per cycle & 360 cycle per revolution
    private final double mm_to_inches = 0.03937008;
    private boolean rounded = true;//toggle to make it more exact
    private final double round_coefficient = 10;//round to the nearest []th

    public double eerp(double t, double degree, double a, double b) {
        return a + (b - a) * Math.pow(t, degree);
    }
    public double deadband(double deadzone, double minval, double input, double degree) {
        int sign = input >= 0 ? 1 : -1;
        if (Math.abs(input) <= deadzone) {
            return 0;
        } else {
            return sign * eerp(Math.abs(input), degree, minval, 1);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Figure out if we're left or right
        Robot robot = new Robot(hardwareMap);

        // --- RESET ALL MOTOR POWERS TO 0 --- //
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);

        waitForStart();

        if (isStopRequested()) return;

        //temporary variables
        int fieldCentricTrigger = 0;
        boolean liftToggled = true;
        robot.motorLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLiftRight.setDirection(DcMotor.Direction.REVERSE);
        robot.motorLiftLeft.setDirection(DcMotor.Direction.REVERSE);

        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y; // Remember, this is reversed!
            double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

            if (rounded) {
                double temp_strafe = (strafe + 0.05) * round_coefficient; // rounds strafing to the nearest []th, which means the driver doesn't have to be as exact when moving straight
                strafe = (int) temp_strafe / round_coefficient;
            }


            double turn = gamepad1.right_stick_x;
            if (gamepad1.right_bumper) {
                robot.isFieldCentric = !robot.isFieldCentric;
            }
            double speedMultiplier = 0.5;

            if (gamepad1.left_bumper && robot.speedMultiplier != speedMultiplier) {
                robot.setSpeedMultipler(speedMultiplier); // reset it back to slow mode
            } else if (gamepad1.left_bumper) {
                robot.setSpeedMultipler(1);
            }


            if (robot.isFieldCentric) {
                robot.fieldCentric(power, strafe, turn);
            } else {
                robot.mecanum(power, strafe, turn);
            }
            if (gamepad1.square) {
                robot.claw(true);
            }

            if (gamepad1.triangle){
                robot.claw(false);
            }


            if (gamepad1.right_trigger>0.3 && robot.motorLiftRight.getCurrentPosition() < 650) {
                robot.motorLiftRight.setPower(0.55);
                robot.motorLiftLeft.setPower(0.55);
            } else if (gamepad1.left_trigger>0.3 && robot.motorLiftRight.getCurrentPosition() > 0) {
                telemetry.addData("down",gamepad1.left_trigger);
                robot.motorLiftRight.setPower(-0.33);
                robot.motorLiftLeft.setPower(-0.33);
            } else {
                // hold if pos is within range, otherwise set to 0 - but will this cause a situation where you can never move the lift?
                if(robot.motorLiftRight.getCurrentPosition()>10 && robot.motorLiftRight.getCurrentPosition()<650){
                    robot.motorLiftRight.setPower(0.05);
                    robot.motorLiftLeft.setPower(0.05);
                }
                else{
                    robot.motorLiftRight.setPower(0);
                    robot.motorLiftLeft.setPower(0);
                }


                /*
                * robot.motorLiftRight.setTargetPosition(robot.motorLiftRight.getCurrentPosition());
                robot.motorLiftLeft.setTargetPosition(robot.motorLiftRight.getCurrentPosition());

                 * */



//
//                if(robot.motorLiftRight.getCurrentPosition()>50){
//                  robot.motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                  robot.motorLiftRight.setTargetPosition(robot.motorLiftRight.getCurrentPosition());
//                }
            }


            //telemetry.addData("Power: ", power);
            //telemetry.addData("Strafe: ", strafe); //0 is straight forward, 1 is straight to the side
            telemetry.addData("IMU Heading: ", -robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Field Centric: ", robot.isFieldCentric);
            telemetry.addData("speed multiplier: ", robot.speedMultiplier);
            telemetry.addLine("Right Lift Position" + Double.toString(robot.motorLiftRight.getCurrentPosition()));
            telemetry.addLine("Left Lift Position" + Double.toString(robot.motorLiftLeft.getCurrentPosition()));
            telemetry.update();
        }
    }
}