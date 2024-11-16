package org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.packages;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class gamepad2_presets {

        private SliderManger SM = new SliderManger();
        private servoManger SO = new servoManger();
        private ClawServo CS = new ClawServo();
        private servoManger ClawRotateServo = new servoManger();

        if(-gamepad2.right_stick_y;

        {
                SM.setPos(675, 0.5);
                sleep(1000);
        }

        if(-gamepad2.left_stick_y;

        {
                SM.setPos(180, 0.5);
        }

        if(gamepad2.a);

        {
                sleep(1000);
                ClawRotateServo.setServoPosition(0.4);
                sleep(1000);
                ClawServo.setServoPosition(0.7);
        }

}