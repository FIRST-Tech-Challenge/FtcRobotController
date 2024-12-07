package org.firstinspires.ftc.teamcode.BBcode;


//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static java.lang.Math.acos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Arm;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Viper;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.WristClaw;

@TeleOp(name = "StateMachineTeleOp")
public class StateMachineTeleOp extends LinearOpMode{
    public enum RobotState{
        HighBasket,
        Home,
        Submersible
    }

    RobotState robotState = RobotState.Home;

    @Override
    public void runOpMode() throws InterruptedException{
        // Initialization Code Goes Here
        TelemetryHelper telemetryHelper = new TelemetryHelper(this);
        //Allows for telemetry to be added to without clearing previous data. This allows setting up telemetry functions to be called in the loop or adding telemetry items within a function and not having it cleared on next loop
        telemetry.setAutoClear(false);
        //Init for the other classes this opmode pulls methods from
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
        Arm arm = new Arm(this, telemetryHelper);
        Viper viper = new Viper(this);
        WristClaw wristClaw = new WristClaw(this);
        telemetryHelper.initGamepadTelemetry(gamepad1);
        telemetryHelper.initGamepadTelemetry(gamepad2);
        //Where the start button is clicked, put some starting commands after
        waitForStart();

        while(opModeIsActive()){ //while loop for when program is active

            //Drive code
            drivetrain.Drive();

            switch(robotState) {
                case Home:
                    //Handle inputs from gamepad
                    if (gamepad2.left_trigger > 0 && gamepad2.dpad_up) {
                        robotState = RobotState.HighBasket;
                    }
                    //left trigger and up dpad switches to HighBasket
                    break;
                case HighBasket:
                    //goal
                    int targetarmposition = 1400;
                    arm.MoveToHighBasket();
                    if (arm.get_armMotor().getCurrentPosition() >= targetarmposition) {
                        viper.ExtendFull(1);
                    }

                    break;
                case Submersible:
                    break;
            }

            telemetry.update();

        }
    }
}