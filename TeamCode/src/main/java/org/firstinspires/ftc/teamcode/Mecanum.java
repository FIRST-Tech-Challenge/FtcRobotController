package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//23477
public class Mecanum extends LinearOpMode {
    private DcMotor Left_Front;
    private DcMotor Right_Front;
    private DcMotor Left_Back;
    private DcMotor Right_Back;
    private DcMotor Arm_Motor;
    private DcMotor Right_Intake;
    private DcMotor Arm1;
    private DcMotor Arm2;
    private Servo LServo;
    private Servo RServo;
    private CRServo intakeServo;
    private Servo armServo;

    // private OpticalDistanceSensor distance_Sensor;
    org.firstinspires.ftc.teamcode.PID PID = new PID(1, 0, 0);
    org.firstinspires.ftc.teamcode.PID PID2 = new PID(1, 0.0, 0.0);
    PID ArmPID = new PID(0.5, 0.0, 0.025);

    @Override
    public void runOpMode() {

        //distance_Sensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Left_Back = hardwareMap.dcMotor.get("Left_Back");
        Right_Back = hardwareMap.dcMotor.get("Right_Back");
        Arm_Motor = hardwareMap.dcMotor.get("Arm_Motor");
        Right_Intake = hardwareMap.dcMotor.get("Right_Intake");
        RServo = hardwareMap.servo.get("RServo");
        Arm1 = hardwareMap.dcMotor.get("Arm1");
        Arm2 = hardwareMap.dcMotor.get("Arm2");
        LServo = hardwareMap.servo.get("LServo");
        intakeServo = hardwareMap.crservo.get("intakeServo");
        armServo = hardwareMap.servo.get("armServo");

        waitForStart();
        long timeElapsed = 0;

//arm 1 is positive 600 max extension
        //arm 2 is -510 max extension
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
            Right_Back.setDirection(DcMotorSimple.Direction.REVERSE);

            timeElapsed = System.currentTimeMillis();
            if (timeElapsed >= 1) {

            }
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;
            double lstick2 = gamepad2.left_stick_y;
            LServo.scaleRange(-180, 180);
            RServo.scaleRange(-180, 180);
            armServo.scaleRange(-180, 180);
            double intakePower = (rightTrigger - leftTrigger);

            PID.setMaxOutput(1);
            PID.setMinOutput(-1);
            PID.setPID(0.003, 0, 0.001);
            PID2.setMaxOutput(1);
            PID2.setMinOutput(-1);
            PID2.setPID(0.003, 0, 0.001);
            PID.updatePID(Arm1.getCurrentPosition());
            PID2.updatePID(Arm2.getCurrentPosition());
            ArmPID.setMaxOutput(1);
            ArmPID.setMinOutput(-1);
            ArmPID.updatePID(Arm_Motor.getCurrentPosition());
        /*    Arm1.setPower(PID.getResult() - 0.001);
            Arm2.setPower(PID.getResult() - 0.001);*/
            Arm_Motor.setPower(ArmPID.getResult());
            Arm2.setDirection(DcMotorSimple.Direction.REVERSE);
            /*Arm1.setPower(-lstick2);
            Arm2.setPower(lstick2);*/

            if (!(rightTrigger == 0)) {
                intakeServo.setPower(1);
            }
            if (!(leftTrigger == 0)) {
                intakeServo.setPower(-1);
            }
            /*
            if (gamepad1.dpad_left){
                intakeServo.setPower(-1);
            }
            else {
                if (gamepad1.dpad_right){
                    intakeServo.setPower(1);
                }
                else
                {
                    intakeServo.setPower(0);
                }
            }

*/
            if (gamepad1.x){
                ArmPID.setSetPoint(0);
            }
            if (gamepad1.y){
                ArmPID.setSetPoint(-90);
            }
            if (gamepad1.dpad_up) {
                LServo.setPosition(-120);
                RServo.setPosition(120);
            }
            if (gamepad1.dpad_down) {
                LServo.setPosition(1);
                RServo.setPosition(-1);
            }
            if (gamepad1.a) {
                Arm2.setPower(1);
                Arm1.setPower(1);

                /*PID.setSetPoint(600);
                PID2.setSetPoint(600);*/
/*
 Arm_Motor.setPower(0.5);*/
                /*armServo.setPosition(90);*/
            } else {
                if (gamepad1.b) {
/*                    PID.setSetPoint(-500);
                    PID2.setSetPoint(271);*/

                    /*Arm_Motor.setPower(-0.5);*/
                } else {

                }
            }




            if (gamepad2.x){
             /*   PID.setSetPoint(-75);*/
            }
            if (gamepad2.y){
/*                PID.setSetPoint(-175);*/
            }
            Right_Intake.setPower(-intakePower);

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

            Left_Front.setPower(frontLeftPower);
            Left_Back.setPower(backLeftPower);
            Right_Front.setPower(frontRightPower);
            Right_Back.setPower(backRightPower);

            telemetry.addLine("Motor Positions");
            telemetry.addData("Front-Left Position", Left_Front.getCurrentPosition());
            telemetry.addData("Front-Right Position", Right_Front.getCurrentPosition());
            telemetry.addData("Back-Left Position", Left_Back.getCurrentPosition());
            telemetry.addData("Back-Right Position", Right_Back.getCurrentPosition());
            telemetry.addData("Arm One Encoder", Arm1.getCurrentPosition());
            telemetry.addData("Arm Two Encoder", Arm2.getCurrentPosition());
            telemetry.addData("Intake Pos", LServo.getPosition());
            telemetry.addData("ArmRotateAbsEncoder", Arm_Motor.getCurrentPosition());

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