
// simple teleop program that drives bot using controller joysticks in tank mode.
// this code monitors the period and stops when the period is ended. Also uses
// controller button A to lower the arm, button B to raise the arm. Controller
// button X opens the gripper and button Y closes the gripper.

package org.firstinspires.ftc.teamcode.huffman;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.util.Range;

@TeleOp()
@Disabled
//@Disabled
public class AllServosAH extends LinearOpMode
{
    DcMotor armMotor;
    Servo   wristServo, left_Gripper, right_Gripper;
    //CRServo contServo;
    float   leftY, rightY;
    double  armPosition, gripPosition, contPower;
    double  MIN_POSITION = 0, MAX_POSITION = 1;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        //rightMotor = hardwareMap.dcMotor.get("right_motor");

        //leftMotor.setDirection(DcMotor.Direction.REVERSE);

        wristServo = hardwareMap.servo.get("Wrist Servo");
        left_Gripper = hardwareMap.servo.get("Left Gripper");
        right_Gripper = hardwareMap.servo.get("Right Gripper");
        //contServo = hardwareMap.crservo.get("cont_servo");

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        armPosition = .5;                   // set arm to half way up.
        gripPosition = MAX_POSITION;        // set grip to full open.

        while (opModeIsActive())
        {
            leftY = gamepad1.left_stick_y * -1;
            rightY = gamepad1.right_stick_y * -1;

            armMotor.setPower(Range.clip(leftY, -1.0, 1.0));
            //rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);

            // check the gamepad buttons and if pressed, increment the appropriate position
            // variable to change the servo location.

            // move arm down on A button if not already at lowest position.
            if (gamepad1.a && armPosition > MIN_POSITION) armPosition -= .01;

            // move arm up on B button if not already at the highest position.
            if (gamepad1.b && armPosition < MAX_POSITION) armPosition += .01;

            // open the gripper on X button if not already at most open position.
            if (gamepad1.x && gripPosition < MAX_POSITION) gripPosition = gripPosition + .01;
            
            // close the gripper on Y button if not already at the closed position.
            if (gamepad1.y && gripPosition > MIN_POSITION) gripPosition = gripPosition - .01;

            // Set continuous servo power level and direction.
            if (gamepad1.dpad_left)
                contPower = .20;
            else if (gamepad1.dpad_right)
                contPower = -.20;
            else
                contPower = 0.0;

            // set the servo position/power values as we have computed them.
            wristServo.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            left_Gripper.setPosition(Range.clip(gripPosition, MIN_POSITION, MAX_POSITION));
            right_Gripper.setPosition(Range.clip(gripPosition, MIN_POSITION, MAX_POSITION));
           // contServo.setPower(contPower);

            telemetry.addData("arm servo", "position=" + armPosition + "  actual=" + wristServo.getPosition());
            telemetry.addData("grip servo", "position=" + gripPosition + "    actual=" + left_Gripper.getPosition());
            telemetry.addData("grip servo", "position=" + gripPosition + "    actual=" + right_Gripper.getPosition());

            //telemetry.addData("arm servo", String.format("position=%.2f  actual=%.2f", armPosition,
            //    armServo.getPosition()));

            //telemetry.addData("grip servo", String.format("position=%.2f  actual=%.2f", gripPosition,
            //    gripServo.getPosition()));

            //telemetry.addData("arm servo", "position=%.2f  actual=%.2f", armPosition, armServo.getPosition());

            //telemetry.addData("grip servo", "position=%.2f  actual=%.2f", gripPosition, gripServo.getPosition());

            telemetry.update();
            idle();
        }
    }
}
