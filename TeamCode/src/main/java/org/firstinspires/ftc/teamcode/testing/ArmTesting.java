package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private DcMotorEx collectionArmMotor;
    private Servo armClaw;

    @Override
    public void init() {
        scoringArmMotor = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        collectionArmMotor = hardwareMap.get(DcMotorEx.class, "horizontalSlide");
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

        scoringArmMotor.setVelocity(yInput * armVelocity * GEAR_RATIO);
        collectionArmMotor.setPower(controller.axis(Controller.Axis.LeftStickX));

        telemetry.addData("Arm Velocity", armVelocity);

        if (controller.pressed(Controller.Button.A))
            armClaw.setPosition(clawClosed ? 0.2 : 0);
    }
}