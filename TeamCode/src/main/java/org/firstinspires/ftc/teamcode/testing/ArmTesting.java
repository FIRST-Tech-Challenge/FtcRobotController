package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;
import org.firstinspires.ftc.teamcode.utils.controller.PowerCurve;

/**
 * This is the class to test the Arm of our robot.
 * */
@TeleOp(name = "ArmTesting", group = "Test Programs")
public class ArmTesting extends OpMode {
    private final GameController controller = new GameController();

    private final int GEAR_RATIO = 60;
    private double armVelocity = 1;

    private DcMotorEx scoringArmMotor;
    private DcMotorEx collectionArmMotor, pivotArm;
    private Servo armClaw;

    @Override
    public void init() {
        scoringArmMotor = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        collectionArmMotor = hardwareMap.get(DcMotorEx.class, "horizontalSlide");

        scoringArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armClaw = hardwareMap.get(Servo.class, "intakeEffector");
        armClaw.setPosition(0);
    }

    boolean clawClosed = true;
    @Override
    public void loop() {
        controller.update(gamepad1);

        float yInput = controller.axis(Controller.Axis.LeftStickY);

        if (gamepad1.dpad_up)
            armVelocity += 0.1;
        else if (gamepad1.dpad_down)
            armVelocity -= 0.1;
        if (!(scoringArmMotor.getCurrentPosition() >= 0 && yInput > 0) && !(scoringArmMotor.getCurrentPosition() <= -2150 && yInput > 0)) {
            scoringArmMotor.setVelocity(yInput * armVelocity * GEAR_RATIO);
        }
        else {
            telemetry.addLine("LIMIT REACHED FOR COLLECTION HARM");
        }
        collectionArmMotor.setPower(controller.axis(Controller.Axis.LeftStickX));

        if (controller.pressed(Controller.Button.B)) {
            collectionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            collectionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (controller.pressed(Controller.Button.Y)) {
            scoringArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scoringArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Arm Velocity", armVelocity);
        telemetry.addData("Scoring position", scoringArmMotor.getCurrentPosition());
        telemetry.addData("Collection position", collectionArmMotor.getCurrentPosition());

        if (controller.pressed(Controller.Button.A))
            armClaw.setPosition(clawClosed ? 0.2 : 0);
    }
}