package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Testing")
public class TestingStuff extends RobotOpMode {
    public float MINIMUM_TRIGGER_PRESS = 0.2f;
    @Override
    public void init() {
        super.init();

        final int NEW_SERVO_POSITION = 180;

        //moveServo(armCatcherServo, NEW_SERVO_POSITION);
    }

    @Override
    public void gamePadMoveRobot() {
        fingerServoPosition = fingerServo.getPosition();
        super.gamePadMoveRobot();
        float FingerPositionModfier = 0;
        if (gamepad1.left_bumper) {
            FingerPositionModfier += 0.1;
        }
        else if (gamepad1.right_bumper) {
            FingerPositionModfier -= 0.1;
        }
        double WristServoPositionModifier = 0;
        if (gamepad1.y) {
            WristServoPositionModifier += 0.1;
        }
        if (gamepad1.x) {
            WristServoPositionModifier -= 0.1;
        }

        elapsedTime.reset();
        while(elapsedTime.seconds() < 0.1) {
            dbp.put("robotLoop", "Waiting");
        }

        moveServo(fingerServo, (float)(fingerServoPosition + FingerPositionModfier));
        fingerServoPosition += FingerPositionModfier;
        moveServo(wristServo, (float)(wristServoPosition + WristServoPositionModifier));

        if (gamepad1.a) {
            armExtensionMotor.setPower(1);
        } else if (gamepad1.b) {
            armExtensionMotor.setPower(-1);
        } else {
            armExtensionMotor.setPower(0);
        }
        
        final float ARM_POWER = gamepad1.left_trigger - gamepad1.right_trigger;
        final float MAX_CLAMPED_ARM_POWER = Math.min(ARM_POWER, MAX_ARM_POWER);
        final float MIN_CLAMPED_ARM_POWER = (MAX_CLAMPED_ARM_POWER < MINIMUM_TRIGGER_PRESS) ? 0 : MAX_CLAMPED_ARM_POWER;
        if (MIN_CLAMPED_ARM_POWER == 0) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(ARM_POWER);
        }
    }

    @Override
    public void robotLoop() {
        gamePadMoveRobot();
    }
}
