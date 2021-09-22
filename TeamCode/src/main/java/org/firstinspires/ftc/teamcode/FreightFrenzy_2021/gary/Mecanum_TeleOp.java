package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.gary;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

@TeleOp(name="practice", group="Linear Opmode")
public class Mecanum_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    double speed = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.right_bumper == true){
                while(gamepad1.right_bumper){
                }
                speed += 0.05;
            }

            if(gamepad1.left_bumper == true){
                while(gamepad1.left_bumper){
                }
                speed -= 0.05;
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            double lF_P;
            double lB_P;
            double rF_P;
            double rB_P;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double dRL = -gamepad1.left_stick_y;
            double drive = -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;

            lF_P = Range.clip(speed*(drive + turn  - dRL), -1.0, 1.0) ;
            rF_P = Range.clip(speed*(drive + turn + dRL), -1.0, 1.0) ;
            lB_P = Range.clip(speed*(drive - turn + dRL), -1.0, 1.0) ;
            rB_P = Range.clip(speed*(drive - turn - dRL), -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftFront.setPower(lF_P);
            rightFront.setPower(rF_P);
            leftBack.setPower(lB_P);
            rightBack.setPower(rB_P);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FrontMotors", "left (%.2f), right (%.2f)", lF_P, rF_P);
            telemetry.addData("BackMotors", "left (%.2f), right (%.2f)", lB_P, rB_P);
            telemetry.addData("Speed:", speed);
            telemetry.update();

        }
    }
}