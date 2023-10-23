package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Testing")
public class TestingStuff extends RobotOpMode {
    @Override
    public void init() {
        super.init();

        final int NEW_SERVO_POSITION = 180;

        moveServo(armCatcherServo, NEW_SERVO_POSITION);
    }
    @Override
    public void robotloop() {
        float ArmServoPositionModifier = 0;
        if (gamepad1.dpad_left) {
            ArmServoPositionModifier += 0.1;
        }
        if (gamepad1.dpad_right) {
            ArmServoPositionModifier -= 0.1;
        }

        elapsedTime.reset();
        while(elapsedTime.seconds() < 0.1) {
            dbp.put("robotLoop", "Waiting");
        }

        moveServo(armCatcherServo, (armCatcherServoPosition + ArmServoPositionModifier));
    }
}
