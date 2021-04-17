//Setup
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

//OpMode code
public class OpModeV1 extends LinearOpMode {

    //hardware setup
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor shooter;
    private DcMotor belt;
    private DcMotor intake;

    private DcMotor arm;

    //code to play once the OpMode is active
    public void runOpMode() {

        //hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        shooter = hardwareMap.get(DcMotor.class, "buzz");
        belt = hardwareMap.get(DcMotor.class, "belt");
        intake = hardwareMap.get(DcMotor.class, "intake");

        arm = hardwareMap.get(DcMotor.class, "arm");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Variable initialization
        double throttle;
        double pivot;
        double armDrive;
        boolean strafeRight;
        boolean strafeLeft;

        double beltPower;
        double shooterPower;
        boolean powerA;
        boolean powerB;
        boolean powerX;
        boolean powerY;
        boolean intakePower;

        double armUp;
        double armDown;

        while (opModeIsActive()) {
            // Read in values from controller
            throttle = gamepad1.right_stick_y;
            pivot = gamepad1.left_stick_x;
            strafeRight = gamepad1.right_bumper;
            strafeLeft = gamepad1.left_bumper;
            armDrive = gamepad1.right_trigger;

            beltPower = gamepad2.left_stick_y;
            shooterPower = gamepad2.right_trigger;
            powerA = gamepad2.a;
            powerB = gamepad2.b;
            powerX = gamepad2.x;
            powerY = gamepad2.y;
            intakePower = gamepad2.left_bumper;

            armUp = gamepad1.right_trigger;
            armDown = gamepad1.left_trigger;

            // Driving
            if (armDrive <= 0.25) {
                if (strafeRight) {
                    frontLeft.setPower(-0.5);
                    backLeft.setPower(0.5);
                    frontRight.setPower(0.5);
                    backRight.setPower(-0.5);
                } else if (strafeLeft) {
                    frontLeft.setPower(0.5);
                    backLeft.setPower(-0.5);
                    frontRight.setPower(-0.5);
                    backRight.setPower(0.5);
                } else {
                    frontLeft.setPower(throttle);
                    frontRight.setPower(throttle);
                    backLeft.setPower(throttle);
                    backRight.setPower(throttle);

                    frontLeft.setPower(-pivot);
                    frontRight.setPower(pivot);
                    backLeft.setPower(-pivot);
                    backRight.setPower(pivot);
                }
            }else if (armDrive > 0.25){
                if (strafeRight) {
                    frontLeft.setPower(0.5);
                    backLeft.setPower(-0.5);
                    frontRight.setPower(-0.5);
                    backRight.setPower(0.5);
                } else if (strafeLeft) {
                    frontLeft.setPower(-0.5);
                    backLeft.setPower(0.5);
                    frontRight.setPower(0.5);
                    backRight.setPower(-0.5);
                } else {
                    frontLeft.setPower(-throttle);
                    frontRight.setPower(-throttle);
                    backLeft.setPower(-throttle);
                    backRight.setPower(-throttle);

                    frontLeft.setPower(pivot);
                    frontRight.setPower(-pivot);
                    backLeft.setPower(pivot);
                    backRight.setPower(-pivot);
                }
            }

            if (shooterPower > 0){
                if (powerA){
                    shooter.setPower(1);
                }else if (powerB){
                    shooter.setPower(0.8);
                }else if (powerX){
                    shooter.setPower(0.5);
                }else if (powerY){
                    shooter.setPower(0.75);
                }
            }else{
                shooter.setPower(0);
            }
            if (intakePower){
                intake.setPower(.5);
            }else{
                intake.setPower(0);
            }
            if (beltPower > 0){
                belt.setPower(0.5);
            }else if(beltPower<0){
                belt.setPower(-0.5);
            }else{
                belt.setPower(0);
            }

            if (armUp > 0 && armUp < 0.5){
                arm.setPower(armUp);
            }else if (armUp >= 0.5){
                arm.setPower(0.5);
            }else if (armDown > 0){
                arm.setPower(-armDown);
            }else{
                arm.setPower(0);
            }
            telemetry.addData("armUp", armUp);
            telemetry.addData("armDown", armDown);
            telemetry.update();
        }
    }
}
