package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class manualDrive extends LinearOpMode{
    @Override
    public void runOpMode() {
        // make a drive controller instance
        DriveController driveController = new DriveController(hardwareMap);
        Vector joystick_left = new Vector(0, 0);

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        while (opModeIsActive()) {
            // get the movement direction and speed
            joystick_left.x =  gamepad1.left_stick_x;
            joystick_left.y = -gamepad1.left_stick_y;

            // make the bot field oriented while considering the starting angle
            joystick_left.addAngle(-driveController.getRobotAngle() - AutonomousDrive.lastAngle);

            // slow mode if the right bumper is pressed
            if (gamepad1.right_bumper) driveController.overallDrivingPower = 0.4;

            // move the bot with 70% turning speed
            driveController.mecanumDrive(joystick_left, gamepad1.right_stick_x * 0.7);

            // move elevator based on button presses
            if      (A_pressed()) driveController.setElevatorPosition(elevatorPositions.high  );
            else if (X_pressed()) driveController.setElevatorPosition(elevatorPositions.middle);
            else if (Y_pressed()) driveController.setElevatorPosition(elevatorPositions.low   );

            // release the cone if the right trigger is pressed
            else if (right_trigger_pressed() ) driveController.scoreCone();
            // move the elevator back if the right trigger is released
            else if (right_trigger_released()) driveController.setElevatorPosition(elevatorPositions.bottom);

            // temp
            driveController.moveArm(gamepad1.left_trigger);

            // make the commands take effect
            driveController.update();
        }
    }
    private boolean a = true;
    public boolean A_pressed(){
        if (gamepad1.a){
            if(a){
                a = false;
                return true;
            }
        } else a = true;
        return false;
    }

    private boolean x = true;
    public boolean X_pressed(){
        if (gamepad1.x){
            if(x){
                x = false;
                return true;
            }
        } else x = true;
        return false;
    }

    private boolean y = true;
    public boolean Y_pressed(){
        if (gamepad1.y){
            if(y){
                y = false;
                return true;
            }
        } else y = true;
        return false;
    }


    private boolean right_trigger1 = false;
    public boolean right_trigger_pressed(){
        if (gamepad1.right_trigger > 0){
            if(!right_trigger1){
                right_trigger1 = true;
                return true;
            }
        } else right_trigger1 = false;
        return false;
    }

    private boolean right_trigger_was_pressed = false;
    public boolean right_trigger_released(){
        if (gamepad1.right_trigger == 0){
            if(right_trigger_was_pressed){
                right_trigger_was_pressed = false;
                return true;
            }
        } else right_trigger_was_pressed = true;
        return false;
    }
}