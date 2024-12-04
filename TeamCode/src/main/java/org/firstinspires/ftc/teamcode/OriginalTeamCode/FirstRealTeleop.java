package org.firstinspires.ftc.teamcode.OriginalTeamCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Teleop To Use :))))",group = "Teleops to use :))))))")
public class FirstRealTeleop extends LinearOpMode{

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //other motors
    DcMotor armLifterLeft = null;
    DcMotor armLifterRight = null;
    DcMotor armRotate = null;
    DcMotor linearActuator = null;
    //servos
    CRServo spool = null;
    Servo wrist = null;
    CRServo sampPickUpLeft = null;
    CRServo sampPickUpRight = null;


    void linearActuatorMover(int i){
        linearActuator.setTargetPosition(i);
    }

    void linearActuatorRaiser(){
        linearActuatorMover(60);
    }

    void linearActuatorLower(){
        linearActuatorMover(20);
    }



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        leftBackDrive = hardwareMap.dcMotor.get("leftRear");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        rightBackDrive = hardwareMap.dcMotor.get("rightRear");
        armLifterLeft = hardwareMap.dcMotor.get("armLifterLeft");
        armLifterRight = hardwareMap.dcMotor.get("armLifterRight");
        armRotate = hardwareMap.dcMotor.get("armRotate");
        linearActuator = hardwareMap.dcMotor.get("linearActuator");
        spool = hardwareMap.crservo.get("spool");
        wrist = hardwareMap.servo.get("wrist");
        sampPickUpLeft = hardwareMap.crservo.get("sampPickUpLeft");
        sampPickUpRight = hardwareMap.crservo.get("sampPickUpRight");

        sampPickUpLeft.setDirection(CRServo.Direction.FORWARD);
        sampPickUpRight.setDirection(CRServo.Direction.REVERSE);

        armLifterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        boolean isGrabbing = false;
        double wristPos = 0;
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double pose = drive.getExternalHeading(); // used for my custom function to drive straight thru the trusses

            double slowMode = gamepad1.left_trigger;
            double slowCoeff = 0.3;

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            if (slowMode > 0.2) {

                leftFrontDrive.setPower(leftFrontPower * slowCoeff);
                rightFrontDrive.setPower(rightFrontPower * slowCoeff);
                leftBackDrive.setPower(leftBackPower * slowCoeff);
                rightBackDrive.setPower(rightBackPower * slowCoeff);

            } else {

                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

            }
//            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            armRotate.setPower(gamepad2.left_stick_y);
//            wrist.setPosition(-(gamepad2.right_stick_y));
//
//            armLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            armLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            wrist.setPosition(wristPos);
//            wristPos += (Math.pow(gamepad2.right_stick_y,3));
//
//            if(gamepad2.dpad_up){
//                linearActuatorRaiser();
//            } else if(gamepad2.dpad_down){
//                linearActuatorLower();
//            }
//
//            int spoolPow = 0;
//            if(gamepad2.dpad_up){
//                spoolPow = 1;
//            } else if(gamepad2.dpad_down){
//                spoolPow = -1;
//            }
//
//            armLifterLeft.setPower(gamepad2.right_stick_x);
//            armLifterRight.setPower(gamepad2.right_stick_x);
//
//
//            if(isGrabbing){
//                sampPickUpRight.setPower(1);
//                sampPickUpLeft.setPower(1);
//            } else{
//                sampPickUpRight.setPower(0);
//                sampPickUpLeft.setPower(0);
//            }
//            if (gamepad2.left_bumper){
//                isGrabbing = !isGrabbing;
//            }
//
//            if(gamepad2.right_bumper){
//                sampPickUpLeft.setPower(-1);
//                sampPickUpRight.setPower(-1);
//            }
//            spool.setPower(spoolPow);

            //arm rotate
            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRotate.setPower(gamepad2.left_stick_y);

            //actuator
            if(gamepad2.dpad_up){
                linearActuatorRaiser();
            } else if(gamepad2.dpad_down){
                linearActuatorLower();
            }

            //extenders



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
           // telemetry.addData("Controls:\nGAMEPAD 2:\nwrist: right stick y\nspool: right stick x\ntoggle grab: left shoulder\nlinear actuator: up/down dpad\narm rotate: left stick y\nGAMEPAD 1\n lateral movement: left stick xy\n, rotate: right stick x","apple");
            telemetry.update();
        }
    }

}