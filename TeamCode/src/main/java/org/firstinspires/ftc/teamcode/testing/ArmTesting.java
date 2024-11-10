package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;

/**
 * This is the class to test the Arm of our robot.
 * */
@TeleOp(name = "ArmTesting", group = "Test Programs")
public class ArmTesting extends OpMode {
    private final GameController controller2 = new GameController();

    private final int GEAR_RATIO = 60;
    private double armVelocity = 8;

    private DcMotorEx scoringArmMotor;
    private DcMotorEx collectionArmMotor, endPivotMotor;
    private Servo armClaw, bucket;

    @Override
    public void init() {
        scoringArmMotor = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        collectionArmMotor = hardwareMap.get(DcMotorEx.class, "horizontalSlide");
        endPivotMotor = hardwareMap.get(DcMotorEx.class, "endPivotMotor");

        scoringArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        endPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armClaw = hardwareMap.get(Servo.class, "intakeEffector");
        bucket = hardwareMap.get(Servo.class, "bucket");
        armClaw.setPosition(0);
        bucket.setPosition(0);
    }

    boolean clawClosed = true;
    boolean bucketUp = true;
    @Override
    public void loop() {
        controller2.update(gamepad2);

        float yInput = controller2.axis(Controller.Axis.LeftStickY);
        float xInput = controller2.axis(Controller.Axis.RightStickX);

        if (collectionArmMotor.getCurrentPosition() >= 0 && xInput > 0 || collectionArmMotor.getCurrentPosition() <= -2150 && xInput < 0) {
            collectionArmMotor.setVelocity(0);
            telemetry.addLine("LIMIT REACHED FOR COLLECTION ARM");
        }
        else collectionArmMotor.setVelocity(xInput * 500);

        scoringArmMotor.setPower(yInput * armVelocity * GEAR_RATIO);

        if (controller2.pressed(Controller.Button.B)) {
            collectionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            collectionArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (controller2.pressed(Controller.Button.Y)) {
            scoringArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scoringArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double pivot = (controller2.pressed(Controller.Button.DPadUp) ? 1 : 0) + (controller2.pressed(Controller.Button.DPadDown) ? -1 : 0);
        endPivotMotor.setVelocity(pivot * 300);
        telemetry.addData("Arm Velocity", armVelocity);
        telemetry.addData("Scoring position", scoringArmMotor.getCurrentPosition());
        telemetry.addData("Collection position", collectionArmMotor.getCurrentPosition());
        telemetry.addData("Pivot", endPivotMotor.getCurrentPosition());

        if (controller2.pressed(Controller.Button.A)) {
            armClaw.setPosition(clawClosed ? 1 : 0);
            clawClosed = !clawClosed;
        }
        if (controller2.pressed(Controller.Button.X)) {
            bucket.setPosition(bucketUp ? 1 : 0);
            bucketUp = !bucketUp;
        }
    }
}