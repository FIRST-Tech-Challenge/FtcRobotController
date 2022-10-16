package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        DcMotorEx motorLift = hardwareMap.get(DcMotorEx.class, "motorLift");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
        CRServo servoIntake = hardwareMap.crservo.get("intake");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Motor Configuration Settings
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE); //motor direction
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Braking behavior
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //We don't want to use PID for the motors using the encoders

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLift.setTargetPosition(0);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoIntake.setDirection(DcMotor.Direction.FORWARD);

        // --- RESET ALL MOTOR POWERS TO 0 --- //
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorLift.setPower(0);

        Drive drive = new Drive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //temporary variables
        int fieldCentricTrigger = 0;
        int targetLiftPosition = 0;
        boolean liftAuton = false;
        boolean liftToggled = true;
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

            if(gamepad2.left_bumper)
            {
                liftToggled = !liftToggled;
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
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetLiftPosition = 1;
                liftAuton = true;
                motorLift.setTargetPosition(targetLiftPosition);
            }
            if(gamepad2.y)//low
            {
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetLiftPosition = 2;
                liftAuton = true;
                motorLift.setTargetPosition(targetLiftPosition);
            }
            /**
            double sigma = 1;
            if(liftAuton && Math.abs(motorLift.getCurrentPosition() - targetLiftPosition) <= sigma)
            {
                liftAuton = false;
            }
             **/
            if(Math.abs(gamepad2.right_stick_y) > 0.1)
            {
                liftAuton = false;
                motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorLift.setPower(-gamepad2.right_stick_y*0.40);
            }
            else if(liftAuton)
            {
                motorLift.setPower(0.75);
            }
            else if(liftToggled)
            {
                motorLift.setTargetPosition(motorLift.getCurrentPosition());
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            }
            servoIntake.setPower(gamepad2.right_trigger > 0.80 ? 0.79 : gamepad2.right_trigger);
            servoIntake.setPower(gamepad2.left_trigger>0.80 ? -0.79 : -gamepad2.left_trigger);

//            if(gamepad1.b) {
//                motorFrontLeft.setPower(200);
//            }
//            if(gamepad1.a) {
//                motorBackLeft.setPower(200);
//            }
//            if(gamepad1.x) {
//                motorFrontRight.setPower(200);
//            }
//            if(gamepad1.y) {
//                motorBackRight.setPower(200);
//            }

            telemetry.addData("gamepad trigger", gamepad2.right_trigger);
            telemetry.addData("Power: ", power);
            telemetry.addData("Strafe: ", strafe);//0 is straight forward, 1 is straight to the side
            telemetry.addData("IMU Heading: ", -imu.getAngularOrientation().firstAngle);
            telemetry.addData("Field Centric: ", drive.isFieldCentric);
            telemetry.addData("Lift Encoder Position", motorLift.getCurrentPosition());
            telemetry.addData("Lift Brake Toggled: ", liftToggled);
            //telemetry.addData("LiftAuton On?: ", liftAuton);
            //telemetry.addData("LiftAuton: ", targetLiftPosition);
            telemetry.update();
        }
    }
}
