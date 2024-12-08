package org.firstinspires.ftc.teamcode.BBcode;


//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Arm;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Viper;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.WristClaw;

@TeleOp(name = "StateMachineTeleop")
public class StateMachineTeleop extends LinearOpMode{
    enum RobotState {
        Home,
        RisingArmSample,
        ViperExtendFull,
        WristDump,
        HighBasket,
        WristUp,
        ViperClosed,
        ViperRetractedShort,
        LoweringArm
    }

    RobotState robotState = RobotState.Home;

    ElapsedTime wristTimer = new ElapsedTime();

    final double wristFlipTime = 0.75;

    private void handleGamepad2 (WristClaw wristClaw) {

        //Open Claw
        if(gamepad2.b) {
            telemetry.update();
            wristClaw.OpenClaw();
        }

        //Close Claw
        if(gamepad2.x) {
            telemetry.update();
            wristClaw.CloseClaw();
        }

        //Move Claw Up
        if(gamepad2.y) {
            wristClaw.WristUp();
        }

        //Move Claw Down
        if(gamepad2.a) {
            wristClaw.WristDown();
        }
    }

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
        arm.Reset();
        viper.StopAndResetEncoder();
        wristTimer.reset();
//        wristClaw.WristInit();
//        wristClaw.CloseClaw();

        //Call the function to initialize telemetry functions
//        telemetryHelper.initMotorTelemetry( viperMotor, "viperMotor");
        telemetryHelper.initGamepadTelemetry(gamepad1);
        telemetryHelper.initGamepadTelemetry(gamepad2);
        //Where the start button is clicked, put some starting commands after
        waitForStart();
        arm.MoveToHome();

        while(opModeIsActive()){ //while loop for when program is active


            //Drive code
            drivetrain.Drive();

            handleGamepad2(wristClaw);

            switch (robotState) {
                case Home:
                    if (gamepad2.left_trigger > 0 && gamepad2.dpad_up) {
                        arm.MoveToHighBasket();
                        robotState = RobotState.RisingArmSample;
                    }
                    break;
                case RisingArmSample:
                    if (arm.getIsArmHighBasketPosition()) {
                        viper.ExtendFull(1);
                        robotState = RobotState.ViperExtendFull;
                    }
                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        arm.MoveToHome();
                        robotState = RobotState.LoweringArm;
                    }
                    break;

                case ViperExtendFull:
                    if (viper.getIsViperExtendFull()) {
                        wristClaw.WristDump();
                        robotState = RobotState.WristDump;
                        wristTimer.reset();
                    }
                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        viper.ExtendShort(1);
                        robotState = RobotState.ViperRetractedShort;
                    }
                    break;

                case WristDump:
                    if (wristTimer.seconds() >= wristFlipTime){
                        robotState = RobotState.HighBasket;
                    }
                    else if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        wristClaw.WristUp();
                        robotState = RobotState.WristUp;
                        wristTimer.reset();
                    }
                    break;

                case HighBasket:
                    if (gamepad2.left_trigger > 0 && gamepad2.dpad_down) {
                        wristClaw.WristUp();
                        robotState = RobotState.WristUp;
                        wristTimer.reset();
                    }
                    break;

                case WristUp:
                    if (wristTimer.seconds() >= wristFlipTime) {
                        viper.ExtendShort(1);
                        robotState = RobotState.ViperRetractedShort;
                    }
                    break;

                case ViperRetractedShort:
                    if (viper.getIsViperRetractedShort()) {
                        viper.ExtendClosed(0.25);
                        robotState = RobotState.ViperClosed;
                    }

                case ViperClosed:
                    if (viper.getIsViperExtendClosed()) {
                        arm.MoveToHome();
                        robotState = RobotState.LoweringArm;
                    }
                    break;

                case LoweringArm:
                    if (arm.getIsArmHomePosition()) {
                        robotState = RobotState.Home;
                    }
                    break;
            }

            telemetry.update();

        }
    }
}