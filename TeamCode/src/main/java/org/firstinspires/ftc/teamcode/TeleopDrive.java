package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.tools.SetDriveMotors;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FC_square_TO_drive", group = "Testing")
public class TeleopDrive extends LinearOpMode {
    private SetDriveMotors setDriveMotorsObj;

    Robot robot;

    public void Setup(){
        TelemetryManager.setTelemetry(telemetry);
        setDriveMotorsObj = new SetDriveMotors(hardwareMap, gamepad1);

        robot = new Robot(hardwareMap, gamepad1, gamepad2);

    }

    public boolean atRest(){
        return(
                Math.abs(gamepad1.left_stick_y) < setDriveMotorsObj.DEADZONE_MIN_Y &&
                        Math.abs(gamepad1.right_stick_y) < setDriveMotorsObj.DEADZONE_MIN_Y &&
                        Math.abs(gamepad1.left_stick_x) < setDriveMotorsObj.DEADZONE_MIN_X &&
                        Math.abs(gamepad1.right_stick_x) < setDriveMotorsObj.DEADZONE_MIN_X);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();
        while(opModeIsActive()){
//            if(drive.isBusy()&& atRest()){
//                drive.update();
//            }
            telemetry.update();
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean goFast = gamepad1.left_bumper;
//            if (!drive.isBusy() || !atRest()) {
//                setMotorsObj.driveCommands(hardwareMap, horizontal, vertical, turn, goFast);
//            }
            setDriveMotorsObj.driveCommands(horizontal, vertical, turn, goFast);


            robot.update();

            if(robot.currentState()== robot.outTakingPixels){

                if(Robot.handlerRightBumper.Pressed()){
                    Robot.clawGrip.setPosition(Robot.clawOpen);
                }

                if(Robot.handlerDPad_Left.Pressed()){
                    Robot.clawYaw.setPosition(Robot.clawYawLeft);
                }
                if(Robot.handlerDPad_Down.Pressed()){
                    Robot.clawYaw.setPosition(Robot.clawYawIntake);
                }
                if(Robot.handlerDPad_Right.Pressed()){
                    Robot.clawYaw.setPosition(Robot.clawYawRight);
                }
            }

        }
    }

}