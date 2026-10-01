package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "CodyTake", group = "TeleOp")
public class CodyTake extends LinearOpMode {

    // Declare hardware variables
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private CRServo axonCRServo;

    @Override
    public void runOpMode() {

        leftMotor   = hardwareMap.get(DcMotor.class, "LM");
        rightMotor  = hardwareMap.get(DcMotor.class, "RM");
        axonCRServo = hardwareMap.get(CRServo.class, "NTK");


        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {


            double leftPower  = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.left_stick_y;

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);



            if (gamepad1.dpad_up) {
                axonCRServo.setPower(1.0);
            } else if (gamepad1.dpad_down) {
                axonCRServo.setPower(-1.0);
            } else {
                axonCRServo.setPower(0.0);
            }



            telemetry.addData("Left Motor Power", leftPower);
            telemetry.addData("Right Motor Power", rightPower);
            telemetry.addData("Axon Servo Power", axonCRServo.getPower());
            telemetry.update();
        }
    }
}