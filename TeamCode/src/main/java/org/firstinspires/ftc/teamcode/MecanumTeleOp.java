package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

@TeleOp(name="DriveOfficial")
public class MecanumTeleOp extends LinearOpMode {
    private final double inches_per_revolution = 60/25.4*Math.PI; //60 mm * (1 inches)/(25.4 mm) is the diameter of the wheel in inches, *pi for circumference
    private final double ticks_per_revolution = 360*6.0; //4 ticks per cycle & 360 cycle per revolution
    private final double mm_to_inches = 0.03937008;
    private boolean rounded = true;//toggle to make it more exact
    private final double round_coefficient = 10;//round to the nearest []th

    @Override
    public void runOpMode() throws InterruptedException {
        //Figure out if we're left or right



        // --- DECLARE DC MOTORS FOR DRIVE --- //
        DcMotorEx leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotorEx rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Motor Configuration Settings
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); //motor direction
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Braking behavior
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //We don't want to use PID for the motors using the encoders

        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // --- RESET ALL MOTOR POWERS TO 0 --- //
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
<<<<<<< Updated upstream
=======
        leftLift.setPower(0);
        rightLift.setPower(0);
>>>>>>> Stashed changes




        Drive drive = new Drive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, imu);


        waitForStart();

        if (isStopRequested()) return;

        //temporary variables
        int fieldCentricTrigger = 0;
        int targetLiftPosition = 0;
        boolean liftAuton = false;
        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y; // Remember, this is reversed!
            double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            if(rounded)
            {
                double temp_strafe = (strafe+0.05)*round_coefficient;//rounds strafing to the nearest []th, which means the driver doesn't have to be as exact when moving straight
                strafe = (int)temp_strafe/round_coefficient;
            }
            double turn = gamepad1.right_stick_x;
            if(gamepad1.right_bumper) {
                if(fieldCentricTrigger >= 20){
                    drive.isFieldCentric = !drive.isFieldCentric;
                }
                fieldCentricTrigger++;
            }
            else{fieldCentricTrigger = 0;}


            if(drive.isFieldCentric) {
                drive.fieldCentric(power, strafe, turn);
            }
            else {
                drive.mecanum(power, strafe, turn);
            }
            //Lift Stuff
            if(gamepad2.back)//turn off liftAuton
            {
                liftAuton = false;
            }
            if(gamepad2.x)//mid
            {
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetLiftPosition = 1;
                liftAuton = true;
                leftLift.setTargetPosition(targetLiftPosition);
                rightLift.setTargetPosition(targetLiftPosition);
            }
            if(gamepad2.y)//low
            {
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetLiftPosition = 2;
                liftAuton = true;
                leftLift.setTargetPosition(targetLiftPosition);
                rightLift.setTargetPosition(targetLiftPosition);
            }
            if(gamepad2.right_stick_y != 0)
            {
                liftAuton = false;
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLift.setPower(gamepad2.right_stick_y);
                rightLift.setPower(gamepad2.right_stick_y);
            }
            else if(liftAuton)
            {
                leftLift.setPower(0.75);
                rightLift.setPower(0.75);

            }


            telemetry.addData("Power: ", power);
            telemetry.addData("Strafe: ", strafe);//0 is straight forward, 1 is straight to the side
            telemetry.addData("IMU Heading: ", -imu.getAngularOrientation().firstAngle);
            telemetry.addData("Field Centric: ", drive.isFieldCentric);
            telemetry.addData("LiftAuton On?: ", liftAuton);
            telemetry.addData("LiftAuton: ", targetLiftPosition);
            telemetry.update();
        }
    }
}
