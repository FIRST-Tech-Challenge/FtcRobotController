package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="Robot: Test Scaffold", group="Robot")
public class TestTeleOp extends LinearOpMode {
    public BrainStemRobot robot;
    public final double SLOW = 0.5;
    public void runOpMode(){

        robot = new BrainStemRobot(hardwareMap, telemetry);
        Turret turret = new Turret(hardwareMap, telemetry);
        robot.initializeRobotPosition();


        // Preset positions for turret via buttons x, y, b
        if(gamepad1.y){
            robot.turret.moveToPreset(turret.TURRET_LEFT_POSITION, robot.lift);
        }
        if(gamepad1.b){
            robot.turret.moveToPreset(turret.TURRET_RIGHT_POSITION, robot.lift);
        }
        if(gamepad1.x){
            robot.turret.moveToPreset(turret.TURRET_CENTER_POSITION, robot.lift);
        }

        // operator controlled lift via left stick y axis
        if(gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2){
            robot.lift.setMotor((gamepad1.left_stick_y * SLOW));
        }
        else{
            robot.lift.setMotor(0);
        }

        // operator controlled turret via right stick x axis
        if(gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2){
            robot.turret.setMotor((gamepad1.right_stick_x * SLOW));
        }
        else{
            robot.turret.setMotor(0);
        }

        // operator controlled extension via right stick y axis
        if(gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2){
            robot.arm.extend(gamepad1.right_stick_y);   // extend adjusts move speed, no need for SLOW multiplier
        }
        else{
            robot.arm.extend(0);
        }

        // TODO: The following segment will not work. Need to latch the position until next time the trigger is clicked.
        // Grab or drop the cone via right trigger (toggle)
        if (gamepad1.right_bumper) {
            if (robot.arm.grabberPosition() > robot.arm.GRABBER_POSITION_OPEN)
                robot.arm.grabberOpen();
            else
                robot.arm.grabberHold();
        }

        // Tilt the grabber via left trigger; hold to keep it tilted, release to reset tilt
        if (gamepad1.left_bumper) {
            robot.arm.tiltUp();
        }
        else {
            robot.arm.tiltDown();
        }
        telemetry.addData("Lift" , robot.lift.getPosition());
        telemetry.addData("Turret", robot.turret.getPosition());
        telemetry.addData("Extension", robot.arm.getExtensionPosition());

        telemetry.update();


    }

}
