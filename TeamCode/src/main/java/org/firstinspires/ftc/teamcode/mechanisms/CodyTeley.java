package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import fi.iki.elonen.NanoHTTPD;
@TeleOp(name = "CodyTeley", group = "TeleOp")
public class CodyTeley {
    private DcMotorEx myMotor;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, IntakeMotor,ShooterMotor;
    private CRServo ContinuousServo;
    MecanumDrive drive = new MecanumDrive();
    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "FL");
        backLeftMotor = hwMap.get(DcMotor.class, "BL");
        frontRightMotor = hwMap.get(DcMotor.class, "FR");
        backRightMotor = hwMap.get(DcMotor.class, "BR");
        ContinuousServo = hardwareMap.get(CRServo.class, "RigbtNTKServo");
        ContinuousServo = hardwareMap.get(CRServo.class, "LeftNTKServo");
        IntakeMotor = hwMap.get(DcMotor.class,"NTK");
        ShooterMotor =hwMap.get(DcMotor.class,"Shooter");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward - strafe - rotate;
        double backLeftPower = forward + strafe - rotate;
        double frontRightPower = forward + strafe + rotate;
        double backRightPower = forward - strafe + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;


        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        class Cody extends LinearOpMode {
            private DcMotorEx myMotor;

            @Override
            public void runOpMode() {
                myMotor = hardwareMap.get(DcMotorEx.class, "myMotor");
                myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                waitForStart();

                while (opModeIsActive()) {
                    double y  = gamepad1.left_stick_y;
                    double x = -gamepad1.left_stick_x * 1.1;
                    double rx = -gamepad1.right_stick_x;
                    drive.drive(forward,strafe,rotate);


                    double targetVelocity = -gamepad1.left_stick_y * 2000;
                    myMotor.setVelocity(targetVelocity);

                    telemetry.addData("Target Vel", targetVelocity);
                    telemetry.addData("Actual Vel", myMotor.getVelocity());
                    telemetry.update();

                    ShooterMotor.setDirection(DcMotor.Direction.FORWARD);

                  //telemetry.addData();
                 // telemetry.update();

                    waitForStart();

                    while (opModeIsActive()) {
                        if (gamepad1.b)  {
                            ShooterMotor.setPower(0.0);
                        } else {
                            ShooterMotor.setPower(0.0);
                        }
                    }

                }
                if (gamepad1.b) {
                    IntakeMotor.setPower(1.0);
                } else if (gamepad1.a) {
                    IntakeMotor.setPower(-1.0);
                } else {
                    IntakeMotor.setPower(0.0);
                }
            }
        }
    }
}