package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;

@TeleOp
public class Linear_root extends LinearOpMode {
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private Servo gripper = null;

    @Override
    public void runOpMode() {
        // init chassis
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        Chassis.init(motorFL, motorFR, motorBL, motorBR);

        // init arms
        leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        Arm.init(leftLift, rightLift, gripper);
        Arm.armTarget = 0;

        waitForStart();

        while (opModeIsActive()) {
            Chassis.joyStick(gamepad1);


//            if (gamepad2.left_trigger > 0.2) Arm.manualArm = Arm.ManualArm.drop;
//            else if (gamepad2.right_trigger > 0.2) Arm.manualArm = Arm.ManualArm.raise;
//            else Arm.manualArm = Arm.ManualArm.none;

            if (gamepad2.left_trigger > 0.2) Arm.armTarget = Arm.getCurrentPosition() - 30;
            else if (gamepad2.right_trigger > 0.2) Arm.armTarget = Arm.getCurrentPosition() + 30;

            if (gamepad2.x) Arm.armTarget = Arm.lowJunction;
            if (gamepad2.b) Arm.armTarget = Arm.middleJunction;
            if (gamepad2.y) Arm.armTarget = Arm.highJunction;
            if (gamepad2.a) Arm.armTarget = 0;

            if (gamepad2.left_bumper) Arm.openGripper();
            if (gamepad2.right_bumper) Arm.closeGripper();


            //TODO: SET 1.0 FOR 11166-RC!!
            Arm.setArmPower(1.0);

            telemetry.addData("arm position", Arm.getCurrentPosition());
            telemetry.addData("arm target", Arm.armTarget);
            telemetry.addData("Servo position", Arm.gripper.getPosition());
            telemetry.update();
        }
    }
}
