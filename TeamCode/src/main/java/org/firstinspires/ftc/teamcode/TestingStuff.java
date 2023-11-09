package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Testing")
public class TestingStuff extends RobotOpMode {
    public float MINIMUM_TRIGGER_PRESS = 0.2f;
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void gamePadMoveRobot() {
        dbp.createNewTelePacket();
        //fingerServoPosition = this.fingerServo.getPosition();
        super.gamePadMoveRobot();

        float FingerPositionModfier = 0;
        if (gamepad1.left_bumper) {
            FingerPositionModfier += 0.01;
        }
        else if (gamepad1.right_bumper) {
            FingerPositionModfier -= 0.01;
        }

        float WristServoPositionModifier = 0;
        if (gamepad1.y) {
            log("Test", "Y is pressed");
            WristServoPositionModifier += 0.01;
        }
        else if (gamepad1.x) {
            log("Test", "X is pressed");
            WristServoPositionModifier -= 0.01;
        }

        moveServo(fingerServo, (float)(fingerServoPosition + FingerPositionModfier));
        fingerServoPosition += FingerPositionModfier;

        log("Wrist Pos", String.valueOf(wristServo.getPosition()));
        log("Wrist Pos Modifier", String.valueOf(WristServoPositionModifier));
        if (WristServoPositionModifier != 0) {
            moveServo(wristServo, (float) (wristServoPosition + WristServoPositionModifier));
            wristServoPosition += WristServoPositionModifier;
        }

        if (gamepad1.a) {
            armExtensionMotor.setPower(1);
        } else if (gamepad1.b) {
            armExtensionMotor.setPower(-1);
        } else {
            armExtensionMotor.setPower(0);
        }

        log("ROOT", "Arm Extension: " +
                armExtensionMotor.getCurrentPosition());
        
        final float ARM_POWER = gamepad1.left_trigger - gamepad1.right_trigger;
        final float MAX_CLAMPED_ARM_POWER = Math.min(ARM_POWER, MAX_ARM_POWER);
        final float MIN_CLAMPED_ARM_POWER = (MAX_CLAMPED_ARM_POWER < MINIMUM_TRIGGER_PRESS) ? 0 : MAX_CLAMPED_ARM_POWER;

        log("ROOT", "Min Clamped Arm Power: " + MIN_CLAMPED_ARM_POWER);
        log("ROOT","Max Clamped Arm Power: " + MAX_CLAMPED_ARM_POWER);
        log("ROOT", "Arm Power: " + ARM_POWER);

        if (MAX_CLAMPED_ARM_POWER == 0) {
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setDirection((ARM_POWER < 0) ?
                    DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            armMotor.setPower(Math.abs(ARM_POWER));
        }
    }

    @Override
    public void robotLoop() {
        gamePadMoveRobot();
    }
}
