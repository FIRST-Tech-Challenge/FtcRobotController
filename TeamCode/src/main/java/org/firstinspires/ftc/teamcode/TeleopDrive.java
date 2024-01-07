package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.teamcode.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.SetDriveMotors;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.Global;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpDrive", group = "Testing")
public class TeleopDrive extends LinearOpMode {
    private SetDriveMotors driveMotors;

    private boolean isLiftReset = false;

    Robot robot;
    AprilTagDetection aprilTagDetection;

    public void Setup(){
        Global.telemetry = telemetry;
        driveMotors = new SetDriveMotors(hardwareMap, gamepad1);

        robot = new Robot(hardwareMap, gamepad1, gamepad2, false);

        aprilTagDetection = new AprilTagDetection();
        aprilTagDetection.Setup(hardwareMap, telemetry);

        Robot.clawPitch.setPosition(Robot.clawPitchIntake);
        Robot.clawYaw.setPosition(Robot.clawYawIntake);
        Robot.clawGrip.setPosition(Robot.clawOpen);
        sleep(1000);

        while(!isStarted() && !isStopRequested()){
            if(!isLiftReset){
                Robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Robot.lift.liftMotor.setPower(-1);
                if(Robot.liftTouchDown.isPressed()){
                    Robot.lift.liftMotor.setPower(0);
                    Robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Robot.lift.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    isLiftReset = true;
                }
        }
        }
        AutoDataStorage.comingFromAutonomous = false;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();
        while(opModeIsActive()){

            if(!isLiftReset) {
                Robot.lift.liftMotor.setPower(-1);
                if (Robot.liftTouchDown.isPressed()) {
                    Robot.lift.liftMotor.setPower(0);
                    isLiftReset = true;
                }
            }

            Global.telemetry.update();
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean goFast = gamepad1.left_bumper;
            boolean emergencyBrakeOverride = gamepad1.right_bumper;
            boolean switchDriveMode = gamepad1.b;
            boolean alignToCardinalPoint = gamepad1.a;


            double distanceToWall = 0;
            if (!emergencyBrakeOverride){
                // AprilTag detection of positions if costly
                // Put calculation within if test so it s performed when needed only
                // thus no calc is in emergency override
                distanceToWall = aprilTagDetection.GetDistanceAwayFromTheBackdrop();
            }


            driveMotors.driveCommands(horizontal, vertical, turn, goFast, distanceToWall, switchDriveMode, alignToCardinalPoint);
            driveMotors.update();

            robot.update();

            if(robot.currentState()== robot.outTakingPixels){

                telemetry.addData("Requested position: ", Robot.clawYaw.getPosition());

                if(Robot.handlerDPad_Left.Pressed()){
                    if(Robot.handlerRightTrigger.On()){
                        Robot.clawYaw.setPosition(Robot.clawYawRightHorizontal);
                    }
                    else{
                        Robot.clawYaw.setPosition(Robot.clawYawLeftHorizontal);
                    }
                }

                if(Robot.handlerDPad_Down.Pressed()){
                    if(Robot.handlerRightTrigger.On()){
                        Robot.clawYaw.setPosition(Robot.clawYawRightSlantedUp);
                    }
                    else{
                        Robot.clawYaw.setPosition(Robot.clawYawLeftSlantedDown);
                    }
                }

                if(Robot.handlerDPad_Up.Pressed()){
                    if(Robot.handlerRightTrigger.On()){
                        Robot.clawYaw.setPosition(Robot.clawYawRightSlantedDown);
                    }
                    else{
                        Robot.clawYaw.setPosition(Robot.clawYawLeftSlantedUp);

                    }
                }

                if(Robot.handlerRightBumper.Pressed()){
                    Robot.clawGrip.setPosition(Robot.clawOpen);
                }
                if(Robot.handlerLeftBumper.Pressed()){
                    Robot.clawGrip.setPosition(Robot.clawClose);
                }
            }

        }
    }

}