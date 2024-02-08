package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "OffSeason_Teleop")
public class TeleOp extends LinearOpMode {

    DcMotorEx leftDrive;
    DcMotorEx rightDrive;
    DcMotorEx armMotor;

    Servo servo1, servo2;
    //wegkowogkg

    static final int armTickCount = 1440; // 360 as tick count = 90 degrees
    static final double armQuarterTurn = armTickCount/4;
    static final double armHalfTurn = armTickCount/2;

    public static final double COUNTS_PER_MOTOR_REV    = 1440 ;
    public static final double DRIVE_GEAR_REDUCTION    = 1.0 ;
    public static final double WHEEL_DIAMETER_INCHES   = 0.314961 ;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //19.7498748746487

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");

        armMotor = hardwareMap.get(DcMotorEx.class, "Arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            leftDrive.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            rightDrive.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);

            if(gamepad2.square) {
                armMotor.setTargetPosition((int) (COUNTS_PER_INCH * 12));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
            }

            while(armMotor.isBusy()) {
                telemetry.addData("Position", armMotor.getCurrentPosition());
                telemetry.update();
            }

            if(!armMotor.isBusy()) {
                armMotor.setPower(0);
            }


            if (gamepad1.right_bumper) {
                if (gamepad1.y) {
                    servo1.setPosition(0);
                } else if (gamepad1.a || gamepad1.b) {
                    servo1.setPosition(0.5);
                } else if (gamepad1.x) {
                    servo1.setPosition(1);
                }
            } else if (gamepad1.left_bumper) {
                if (gamepad1.y) {
                    servo2.setPosition(0);
                } else if (gamepad1.a || gamepad1.b) {
                    servo2.setPosition(0.5);
                } else if (gamepad1.x) {
                    servo2.setPosition(1);
                }
            }
            telemetry.addData("Left drive power", leftDrive.getPower());
            telemetry.addData("Right drive power", rightDrive.getPower());
            telemetry.addData("servo1 position", servo1.getPosition());
            telemetry.addData("servo2 position", servo2.getPosition());
            telemetry.addData("Arm power", armMotor.getPower());

            telemetry.update();

        }




        }
    }
