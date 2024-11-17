import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorTelemetry extends LinearOpMode {
    private double TouchPadInput = 0;
    @Override
    public void runOpMode(){

        DcMotor IntakeRight = hardwareMap.dcMotor.get("Intake Right"); // Ehub Port 0 // X Button To Position Automatically? // Joystick Up And Down?
        DcMotor IntakeLeft = hardwareMap.dcMotor.get("Intake Left");   // Ehub Port 1 // ----------------------------------
        DcMotor RightLift = hardwareMap.dcMotor.get("Right Lift");     // Ehub Port 2 // Triangle Button To Delivery Position
        DcMotor LeftLift = hardwareMap.dcMotor.get("Left Lift");       // Ehub Port 3 // ------------------------------------

        IntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        RightLift.setDirection(DcMotorSimple.Direction.REVERSE);     // Reverses the direction the motor turns

        RightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // Resets the position so it sets it's current position to 0
        LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Resets the position so it sets it's current position to 0

        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLift.setTargetPosition(0);     // Makes sure it starts at the set 0
        RightLift.setTargetPosition(0);    // Makes sure it starts at the set 0

        IntakeRight.setTargetPosition(0);
        IntakeLeft.setTargetPosition(0);

        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Sets the mode so we can say to move the motor a certain amount of ticks
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);//Sets the mode so we can say to move the motor a certain amount of ticks

        IntakeRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Sets the motor to be locked when stopped
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Sets the motor to be locked when stopped

        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.touchpad_finger_1){
                TouchPadInput = (gamepad1.touchpad_finger_1_x + 1) / 2;
            }
            telemetry.addData("Touch Pad", TouchPadInput);

            telemetry.addData("Right Intake Position", IntakeRight.getCurrentPosition());
            telemetry.addData("Left Intake Position", IntakeLeft.getCurrentPosition());

            telemetry.addData("Right Lift Position", RightLift.getCurrentPosition());
            telemetry.addData("Left Lift Position", LeftLift.getCurrentPosition());

            telemetry.update();
        }
    }
}
