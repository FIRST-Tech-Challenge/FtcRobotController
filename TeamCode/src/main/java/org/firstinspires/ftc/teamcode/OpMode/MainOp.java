package org.firstinspires.ftc.teamcode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class MainOp extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        //Initialization

        /*
        //Drivechain Motors
        DcMotor wheelFrontLeft = hardwareMap.dcMotor.get("wheelFrontLeft");
        DcMotor wheelFrontRight = hardwareMap.dcMotor.get("wheelFrontRight");
        DcMotor wheelBackLeft = hardwareMap.dcMotor.get("wheelBackLeft");
        DcMotor wheelBackRight = hardwareMap.dcMotor.get("wheelBackRight");

        //Main lin slide Motors
        DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide");

        //Secondary, single motor lin slide used to "launch" drone at the beginning of the game and hoist the robot at the end
        DcMotor liftingLinSlide = hardwareMap.dcMotor.get("liftingLinSlide");

        //Intake (GoGekko Wheel) Servos
        Servo leftIntakeServo = hardwareMap.servo.get("leftIntakeServo");
        Servo rightIntakeServo = hardwareMap.servo.get("rightIntakeServo");

        //Intake (Hinging) Servos
        Servo leftIntakeHingeServo = hardwareMap.servo.get("leftIntakeHingeServo");
        Servo rightIntakeHingeServo = hardwareMap.servo.get("rightIntakeHingeServo");

        //Drone dropping Servo located upon secondary lin slide
        Servo droneServo = hardwareMap.servo.get("droneServo");

        //Controller Joystick Initialization
        double yLeftStickCTLR1 = gamepad1.left_stick_y;
        double xLeftStickCTLR1 = gamepad1.left_stick_x;
        double yRightStickCTLR1 = gamepad1.right_stick_y;
        double xRightStickCTLR1 = gamepad1.right_stick_x;

        double yLeftStickCTLR2 = gamepad2.left_stick_y;
        double xLeftStickCTLR2 = gamepad2.left_stick_x;
        double yRightStickCTLR2 = gamepad2.right_stick_y;
        double xRightStickCTLR2 = gamepad2.right_stick_x;

        //Controller Button Initialization
        //boolean yButtonCTLR1 = gamepad1;
        //boolean yButtonCTLR1 = gamepad1;
        //boolean yButtonCTLR1 = gamepad1;
        //boolean yButtonCTLR1 = gamepad1;

        //boolean yButtonCTLR1 = gamepad1;
        //boolean yButtonCTLR1 = gamepad1;
        //boolean yButtonCTLR1 = gamepad1;
        //boolean yButtonCTLR1 = gamepad1;

        //Running initialization methods
        Drivechain.driveChainInit(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);
        IntakeGekkoWheelServos.gekkoWheelInit(leftIntakeServo, rightIntakeServo);



         */

        DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide");
        leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //DcMotor wheelFrontLeft = hardwareMap.dcMotor.get("wheelFrontLeft");
        //DcMotor wheelFrontRight = hardwareMap.dcMotor.get("wheelFrontRight");
        //DcMotor wheelBackLeft = hardwareMap.dcMotor.get("wheelBackLeft");
        //DcMotor wheelBackRight = hardwareMap.dcMotor.get("wheelBackRight");

        //Drivechain.driveChainInit(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);

        //OpMode
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {

            //Drivechain.drive(gamepad1.left_stick_y, gamepad2.right_stick_x);



            if(gamepad2.dpad_up) {
                leftLinSlide.setPower(0.5);
                rightLinSlide.setPower(-0.5);
                telemetry.addData("leftLin", leftLinSlide.getCurrentPosition());
                telemetry.addData("rightLin", rightLinSlide.getCurrentPosition());
                telemetry.update();
            } else if(gamepad2.dpad_down){
                leftLinSlide.setPower(-0.5);
                rightLinSlide.setPower(0.5);
                telemetry.addData("leftLin", leftLinSlide.getCurrentPosition());
                telemetry.addData("rightLin", rightLinSlide.getCurrentPosition());
                telemetry.update();
            } else {
                leftLinSlide.setPower(0);
                rightLinSlide.setPower(0);
                telemetry.addData("leftLin", leftLinSlide.getCurrentPosition());
                telemetry.addData("rightLin", rightLinSlide.getCurrentPosition());
                telemetry.update();
            }
        }


    }
}
