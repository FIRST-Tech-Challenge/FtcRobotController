package org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.packages;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

public class gamepad2_button_presets {
    private servoManger SO = new servoManger();
    private ClawServo CS = new ClawServo();
    private servoManger clawRotateServo = new servoManger();
    private SliderManger SM = new SliderManger();
    public volatile com. qualcomm. robotcore. hardware. Gamepad gamepad2;
    if (-gamepad2.right_stick_y >= 0.3) {
        SM.setPos(675, 0.5);
    }
    
    if (-gamepad2.left_stick_y >= 0.3) {
        SM.setPos(180, 0.5);
    }
    
    if (gamepad2.a) {
        clawRotateServo.setServoPosition(0.4);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        clawRotateServo.setServoPosition(0.7);
    }
        }
    }
}