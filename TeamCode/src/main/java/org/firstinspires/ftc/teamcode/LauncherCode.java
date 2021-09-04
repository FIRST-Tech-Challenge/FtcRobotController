package org.firstinspires.ftc.teamcode;
//Created by Nihal & Ethan
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class LauncherCode {
    private Blinker expansion_Hub_1;
    ElapsedTime mytimer = new ElapsedTime();

    public static class Launcher {
        public DcMotor LaunchMotor;
        public Servo LaunchServo;
        private boolean launcherOn = false;
        private double maxPower = 0.9;

        public void LauncherToggle() {
            launcherOn = !launcherOn;
        }

        public void LauncherRun(double power) {
            if (launcherOn && power <= 1 && power >= -1) {
                power = power * -maxPower;
                LaunchMotor.setPower(power);
            } else {
                LaunchMotor.setPower(0);
            }
        }

        public void Shoot() {
            LaunchServo.setPosition(0.72);
        }

        public void Reload() {
            LaunchServo.setPosition(0.99);
        }
    }

    public enum LauncherStates {
        Start,
        ButtonPushed,
        ToggleLauncher,
        Pressed,
        Load,
        ResetPosition,
        firsttimer,
        secondtimer
    }
}



