package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Bare TeleOP")
public class BareTeleOp extends LinearOpMode {

    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;
    private DcMotor outtakeRight, outtakeLeft;
    private Servo flipper;

    private CRServo leftIntakeServo, rightIntakeServo;
    private CRServo leftConveyor, rightConveyor, intake, elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        flipper = hardwareMap.servo.get("flipper");

        //launcher
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");

        //intake and conveyor
        intake = hardwareMap.crservo.get("intake");
        leftConveyor = hardwareMap.crservo.get("leftConveyor");
        rightConveyor = hardwareMap.crservo.get("rightConveyor");

        rightConveyor.setDirection(CRServo.Direction.REVERSE);
        intake.setDirection(CRServo.Direction.REVERSE);

        //elevator
        elevator = hardwareMap.crservo.get("elevator");

        //lifting and lowering intake
        leftIntakeServo = hardwareMap.crservo.get("LIrelease");
        rightIntakeServo = hardwareMap.crservo.get("RIrelease");

        rightIntakeServo.setDirection(CRServo.Direction.REVERSE);

        //reverse the needed motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse one of the outtakes
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);

        double powerMod = 1.0;
        double intakeMod = 1.0;

        waitForStart();

        while (opModeIsActive()) {

            //everything intake
            /*
            Change direction of intake
            */
            if(gamepad1.a){//press and hold a while running intake
                intakeMod = -1.0;
            }else{
                intakeMod = 1.0;
            }

            double intakeSpeed = gamepad1.right_trigger * intakeMod;
            intake.setPower(intakeSpeed);
            rightConveyor.setPower(intakeSpeed);//turn conveyor on when the intake turns on
            leftConveyor.setPower(intakeSpeed);

            //Release intake
            if(gamepad1.x){
                lowerIntake(400);
            }
            if(gamepad1.y){
                raiseIntake(400);
            }

            if(gamepad1.dpad_up){
                raiseIntake(100);
            }
            if(gamepad1.dpad_down){
                lowerIntake(100);
            }


            /*
            Checks if right bumper is pressed. If so, power is reduced
             */
            if (gamepad1.right_bumper) {
                powerMod = 0.5;
            } else {
                powerMod = 1.0;
            }

            //Ring elevator
            //Run by a continuous servo; run continuous servo for some amount of time
            if(gamepad2.x){
                raiseElevator(400);
            }
            if(gamepad2.y){
                lowerElevator(400);
            }
            if(gamepad2.dpad_up){
                lowerElevator(100);
            }
            if(gamepad2.dpad_down){
                raiseElevator(100);
            }

            //everything driving
            //Mecanum drive using trig
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI / 4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r * Math.cos(angle);
            double powerTwo = r * Math.sin(angle);

            motorFrontLeft.setPower((powerOne + (rotation))*powerMod);
            motorFrontRight.setPower((powerOne - (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo + (rotation))*powerMod);
            motorBackRight.setPower((powerTwo - (rotation))*powerMod);

            //outtake
            double outtakePower = (gamepad2.right_trigger * -0.52);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);

            //flipper
            if (gamepad2.b) {
                    flipper.setPosition(1);
            }

            if(gamepad2.a){
                flipper.setPosition(0);
            }



        }
    }
    private void raiseIntake(int time){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.milliseconds() < time){
            leftIntakeServo.setPower(1);
            rightIntakeServo.setPower(1);
        }

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
    private void lowerIntake(int time){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.milliseconds() < time){
            leftIntakeServo.setPower(-1);
            rightIntakeServo.setPower(-1);
        }

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
    private void raiseElevator(int time){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.milliseconds() < time){
            elevator.setPower(1);
        }

        elevator.setPower(0);
    }

    private void lowerElevator(int time){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.milliseconds() < time){
            elevator.setPower(-1);
        }

        elevator.setPower(0);
    }
}
