package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class GripperTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Shoulder_Motor = hardwareMap.dcMotor.get("Shoulder_Motor");
        Shoulder_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo Gripper = hardwareMap.servo.get("Gripper_Servo");

//        Servo servoTest = hardwareMap.get(Servo.class, "launch_servo");

        Robot2024 robot2024 = new Robot2024(Gripper, Shoulder_Motor);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            double modifier = 0.5;

//            Shoulder_Motor.setPower((gamepad2.left_stick_x - gamepad2.left_stick_y - gamepad2.right_stick_x) * modifier);
            robot2024.setShoulderPower((gamepad2.left_stick_x - gamepad2.left_stick_y - gamepad2.right_stick_x) * modifier);



            // check to see if we need to move the servo.
            if (gamepad2.b) {
//                closeGripper(Gripper);
                robot2024.closeGripper();
            } else if (gamepad2.a) {
               robot2024.openGripper();
            }
        telemetry.addData("Servo Position", Gripper.getPosition());
//        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Status", "Running");
        telemetry.update();
        }

    }
    public void closeGripper(Servo gripper){
        telemetry.addLine("Closing Gripper");
        gripper.setPosition(0);
    }

    public void OpenGripper(Servo gripper){
        telemetry.addLine("Opening Gripper");
        gripper.setPosition(0.4);
    }
}