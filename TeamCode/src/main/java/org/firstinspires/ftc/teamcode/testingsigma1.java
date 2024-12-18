package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "RambotsPurpleTeleOp")
public class testingsigma1 extends LinearOpMode {

    // Motors for intake and wrist only
    private DcMotor wrist = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;

    private double intakePower = 0;

    @Override
    public void runOpMode() {

        // Initializing hardware for intake and wrist only
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        leftIntake = hardwareMap.get(CRServo.class, "l_intake");
        rightIntake = hardwareMap.get(CRServo.class, "r_intake");

        // Set direction for wrist and intake motors
        wrist.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(CRServo.Direction.FORWARD);
        rightIntake.setDirection(CRServo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

      /*
        GamePad Map
        GamePad 1 (Driver)
          Left JoyStick = lateral, diagonal, forwards, and backwards movements
          Right JoyStick = Rotation of drive train
        GamePad 2 (Operator)
          Button A = toggle position of claw to open or closed (We start closed)
          Left Stick X = wrist control
          Right Stick Y = intake control
       */

            // Wrist control (Operator)
            double wristPower = 0;
            if (gamepad2.left_bumper) {
                wristPower = 1; // Move wrist forward
            } else if (gamepad2.left_trigger > 0) {
                wristPower = -1; // Move wrist backward
            } else {
                wristPower = gamepad2.left_stick_y * 1.10; // Wrist control with the left stick
            }

            // Intake control
            if (gamepad2.a) {
                intakePower = (intakePower == 1 ? 0 : 1); // Toggle intake on or off
            } else if (gamepad2.b) {
                intakePower = (intakePower == -1 ? 0 : -1); // Reverse intake
            }

            // Power to wrist and intake motors
            wrist.setPower(wristPower);
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);

            // Telemetry
            telemetry.addData("Intake power: ", intakePower);
            telemetry.addData("Wrist power: ", wristPower);
            telemetry.update();
        }
    }
}