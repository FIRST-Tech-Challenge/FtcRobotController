//Setup
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    private DcMotor ramp;

    //code to play once the OpMode is active
    public void runOpMode() {

        //hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        shooter = hardwareMap.get(DcMotor.class, "buzz");
        belt = hardwareMap.get(DcMotor.class, "belt");
        ramp = hardwareMap.get(DcMotor.class, "ramp");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Variable initialization
        double throttle;
        double pivot;
        double rampDirection;
        double beltPower;
        double shooterPower;
        boolean strafeRight;
        boolean strafeLeft;

        while (opModeIsActive()){
            // Read in values from controller
            throttle = gamepad1.right_stick_y;
            pivot = gamepad1.left_stick_x;
            beltPower = gamepad2.left_trigger;
            shooterPower = gamepad2.right_trigger;
            rampDirection = gamepad2.right_stick_y;
            strafeRight = gamepad1.right_bumper;
            strafeLeft = gamepad1.left_bumper;

            // Driving
            if (strafeRight){
                frontLeft.setPower(-0.5);
                backLeft.setPower(-0.5);
                frontRight.setPower(-0.5);
                backRight.setPower(0.5);
            }else if (strafeLeft){
                frontLeft.setPower(0.5);
                backLeft.setPower(-0.5);
                frontRight.setPower(-0.5);
                backRight.setPower(0.5);
            }else{
                frontLeft.setPower(throttle);
                frontRight.setPower(throttle);
                backLeft.setPower(throttle);
                backRight.setPower(throttle);

                frontLeft.setPower(-pivot);
                frontRight.setPower(-pivot);
                backLeft.setPower(-pivot);
                backRight.setPower(pivot);
            }

            ramp.setPower(0.25*rampDirection);

            if (shooterPower >= 0.5){
                shooter.setPower(1);
            }else{
                shooter.setPower(shooterPower);
            }

            if (beltPower > 0){
                belt.setPower(0.5);
            }else{
                belt.setPower(0);
            }
        }
    }
}
