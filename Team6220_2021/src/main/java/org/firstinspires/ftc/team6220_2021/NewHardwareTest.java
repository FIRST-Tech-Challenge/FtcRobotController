//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "New Hardware AutonomousTest", group = "TeleOp")
public class NewHardwareTest extends MasterOpMode{
    // Declaring motors and servos
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorLeftDuck;
    DcMotor motorDuck;
    DcMotor motorArm;
    DcMotor motorBelt;
    Servo servoGrabber;
    Servo servoArm;
    int tickvalue = -70;
    double x = 0.7;

    //for run to position or manual control
    boolean toPosition = true;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        //Initialize the motors and servos
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorLeftDuck = hardwareMap.dcMotor.get("motorLeftDuck");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorDuck = hardwareMap.dcMotor.get("motorDuck");
        servoGrabber = hardwareMap.servo.get("servoGrabber");
        servoArm = hardwareMap.servo.get("servoArm");
        motorBelt = hardwareMap.dcMotor.get("motorBelt");
        double position = 0.0;
        double servoArmPostion = 0.0;


        //Set direction of the motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDuck.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set position of servos
        servoGrabber.setPosition(0.34);
        servoArm.setPosition(0.3);

        //Declare variables
        int addingticks = 0;
        boolean isPressed = false;
        double motorPower = 0.9;
        double increase = 1;
        double speed =1;
        int targetPostion = 0;

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Set run mode of arm motor (encoders --> run to position)
        motorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBelt.setTargetPosition(0);
//        motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        //Set power of motors
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                targetPostion = 2800;
            } else if (gamepad1.right_bumper) {
                targetPostion = 0;
            }
            if (gamepad1.left_trigger > 0.9) {
                servoArmPostion += 0.001;
            } else if (gamepad1.right_trigger > 0.9) {
                servoArmPostion -= 0.001;
            }
            servoArm.setPosition(servoArmPostion);
            motorBelt.setPower(gamepad1.right_stick_x);
            //motorBelt.setPower(0.8);
            if (gamepad1.left_stick_y > 0.5) {
                position += 0.001;
            } else if (gamepad1.left_stick_y < -0.5) {
                position -= 0.001;
            }
            if (position > 1){
                position = 1;
            }else if (position < 0){
                position = 0;
            }
            servoGrabber.setPosition(position);
            telemetry.addData("ServoPosition", position);
            telemetry.addData("ServoArmPosition", servoArmPostion);
            telemetry.addData("beltposition", motorBelt.getCurrentPosition());
            telemetry.update();
            motorLeftDuck.setPower(gamepad1.right_stick_y);
        }
    }
}