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
        Chassis chassis = new Chassis(motorFL, motorFR, motorBL, motorBR);
        chassis.init();

        // init arms
        leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        Arm arm = new Arm(leftLift, rightLift, gripper);
        arm.init();
        arm.armTarget = 0;

        waitForStart();

        while (opModeIsActive()) {
            chassis.joyStick(gamepad1);

            if (gamepad2.left_trigger > 0.2) arm.armTarget = arm.getCurrentPosition() - 30;
            else if (gamepad2.right_trigger > 0.2) arm.armTarget = arm.getCurrentPosition() + 30;

            if (gamepad2.x) arm.armTarget = arm.lowJunction;
            if (gamepad2.b) arm.armTarget = arm.middleJunction;
            if (gamepad2.y) arm.armTarget = arm.highJunction;
            if (gamepad2.a) arm.armTarget = 0;

            if (gamepad2.left_bumper){
                arm.openGripper();
                arm.armTarget = 0;
            }
            if (gamepad2.right_bumper) arm.closeGripper();


            //TODO: SET 1.0 FOR 11166-RC!!
            arm.setArmPower(1.0);

            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("arm target", arm.armTarget);
            telemetry.addData("Servo position", arm.gripper.getPosition());
            telemetry.update();
        }
    }
}
