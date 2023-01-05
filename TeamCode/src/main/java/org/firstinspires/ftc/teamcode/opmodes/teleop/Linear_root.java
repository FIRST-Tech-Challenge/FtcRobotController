package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Arm;

@TeleOp
public class Linear_root extends LinearOpMode {
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private CRServo gripper = null;

    @Override
    public void runOpMode() {
        leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        gripper = hardwareMap.get(CRServo.class, "gripper");
        Arm.init(leftLift, rightLift, gripper);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.x) Arm.runToPosition(Arm.lowJunction);
            if (gamepad2.b) Arm.runToPosition(Arm.middleJunction);
            if (gamepad2.y) Arm.runToPosition(Arm.highJunction);
            if (gamepad2.a) Arm.runToPosition(0);
            if (gamepad2.left_bumper) Arm.openGripper();
            if (gamepad2.right_bumper) Arm.closeGripper();
            if (gamepad2.left_trigger > 0) Arm.dropArm();
            if (gamepad2.right_trigger > 0) Arm.raiseArm(0.5);

            telemetry.addData("arm position", Arm.getCurrentPosition());
            telemetry.addData("arm target", Arm.armTarget);
            telemetry.addData("arm power", Arm.getPower());
            telemetry.update();
        }
    }
}
