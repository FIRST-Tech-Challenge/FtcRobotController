package org. firstinspires. ftc. teamcode. PowerPlay23_24;
import org.firstinspires.ftc.teamcode.TemplateJanx;
import static com.qualcomm.robotcore.util.Range.clip;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "RedLeft", group = "PushBot")
public class Blue extends LinearOpMode{
    /* Declare OpMode members. */
    private Servo lc;
    private Servo rc;
    private Servo nodder;
    private DcMotorEx extender;

    private DcMotorEx rotater;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx turn;
    private DcMotorEx ext;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1600.0;
    static final double     TURN_SPEED              = 800.8;

    @Override
    public void runOpMode() {
        initial();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(DRIVE_SPEED,24,24,12); //1
        encoderDrive(TURN_SPEED,16,-20,24); //2 (first turn- right)
        encoderDrive(DRIVE_SPEED,16,16,12); //3
        //going left
        encoderDrive(TURN_SPEED,20,-20,12); //4 (second turn, right)
        encoderDrive(DRIVE_SPEED,20,20,12); //5
        encoderDrive(TURN_SPEED,-20,20,12); //6 (third turn, left)
        //turn right
        encoderDrive(DRIVE_SPEED,10,10,12); //7
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
//    public void open(){
//        //.setPosition()
//     }

//    public void close(){
//        //.setPosition
//    }

    /*
    how to move arm???????
     */
//    public void arm(int degrees){
//       int target = degrees;
//       rotater.setTargetPosition(target);
//       rotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = (backLeft.getCurrentPosition() + frontLeft.getCurrentPosition())/2 + (int)(leftInches * COUNTS_PER_INCH * 60 / 53);
            newRightTarget = (backRight.getCurrentPosition() + frontRight.getCurrentPosition())/2 + (int)(rightInches * COUNTS_PER_INCH * 60 / 53);
            backLeft.setTargetPosition(newLeftTarget);
            frontLeft.setTargetPosition(newLeftTarget);
            backRight.setTargetPosition(newRightTarget);
            frontRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backRight.setPower(Math.abs(speed) * 0.94);
            frontRight.setPower(Math.abs(speed) * 0.94);
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontRight.isBusy() && frontLeft.isBusy())) {
                telemetry.addData("targetL",newLeftTarget);
                telemetry.addData("targetR", newRightTarget);

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Starting at",  "%7d :%7d",
                        frontLeft.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backRight.setPower(0);
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void initial(){
        // Initialize the drive system variables.
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        // janx.armInit("clawLeft","clawRight","nodder","armExtension","arm rotations");
//        ext =       hardwareMap.get(DcMotorEx.class,"armExtension");
//        turn =      hardwareMap.get(DcMotorEx.class,"arm rotations");
//        nodder =    hardwareMap.get(Servo.class,"nodder");
//        lc  =       hardwareMap.get(Servo.class, "clawLeft");
//        rc  =       hardwareMap.get(Servo.class,"clawRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeft.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backRight.getCurrentPosition());
        telemetry.update();


    }

}

