package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
//23477
public class MecanumRED extends LinearOpMode {
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    private DcMotor Left_Back;
    private DcMotor Right_Back;


    // private OpticalDistanceSensor distance_Sensor;
    org.firstinspires.ftc.teamcode.PID PID = new PID(0.1, 0, 0);

    @Override
    public void runOpMode() {

        //distance_Sensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");



        waitForStart();
        long timeElapsed = 0;


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            timeElapsed  = System.currentTimeMillis();
            if (timeElapsed >= 1){

            }
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;

            double intakePower = (rightTrigger - leftTrigger) * 0.75;

            PID.setMaxOutput(1);
            PID.setMinOutput(-1);
            PID.setPID(0.003,0 ,0.001);



            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (Math.abs(gamepad1.left_stick_x) < 0.00001){
                Left_Front.setPower(0);
                Left_Back.setPower(0);
                Right_Front.setPower(0);
                Right_Back.setPower(0);
            }
            if (Math.abs(gamepad1.left_stick_y) < 0.00001){
                Left_Front.setPower(0);
                Left_Back.setPower(0);
                Right_Front.setPower(0);
                Right_Back.setPower(0);
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            Left_Front.setPower(-frontLeftPower);
            Left_Back.setPower(-backLeftPower);
            Right_Front.setPower(frontRightPower);
            Right_Back.setPower(backRightPower);

            telemetry.addLine("Motor Positions");
            telemetry.addData("Front-Left Position", Left_Front.getCurrentPosition());
            telemetry.addData("Front-Right Position", Right_Front.getCurrentPosition());
            telemetry.addData("Back-Left Position", Left_Back.getCurrentPosition());
            telemetry.addData("Back-Right Position", Right_Back.getCurrentPosition());

            telemetry.addLine("");
            telemetry.addLine("Controller Inputs");
            telemetry.addData("Left Trigger", leftTrigger);
            telemetry.addData("Real Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", rightTrigger);
            telemetry.addData("Real Right Trigger", gamepad1.right_trigger);
            telemetry.addData("PID Result", PID.getResult());

            telemetry.addLine("");
            //telemetry.addData("Distance Sensor", distance_Sensor);
            telemetry.update();
        }
    }
}