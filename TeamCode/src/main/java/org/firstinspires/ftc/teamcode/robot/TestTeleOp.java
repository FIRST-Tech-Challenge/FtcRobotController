package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;

@TeleOp(name="Robot: Test Scaffold", group="Robot")
public class TestTeleOp extends LinearOpMode {
//    public BrainStemRobot robot;
    public final double SLOW = 1.0;
    public HashMap stateMap;

    public void runOpMode(){

        BrainStemRobot robot = new BrainStemRobot(hardwareMap, telemetry,stateMap);
        //Turret turret = new Turret(hardwareMap, telemetry);
        //robot.initializeRobotPosition();
        Lift lift = new Lift(hardwareMap, telemetry);
        Extension arm = new Extension(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Preset positions for turret via buttons x, y, b
        /*if(gamepad1.y){
            robot.turret.moveToPreset(turret.TURRET_LEFT_POSITION, robot.lift);
        }
        if(gamepad1.b){
            robot.turret.moveToPreset(turret.TURRET_RIGHT_POSITION, robot.lift);
        }
        if(gamepad1.x){
            robot.turret.moveToPreset(turret.TURRET_CENTER_POSITION, robot.lift);
        }

        // operator controlled lift via left stick y axis

         */
        waitForStart();
        while(!isStopRequested()) {
            if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2) {
                lift.setMotor((gamepad2.left_stick_y * SLOW));
            } else {
                lift.setMotor(0);
            }
            if(gamepad2.y){
                stateMap.put(robot.LIFT_SYSTEM_NAME,robot.LIFT_POLE_LOW);
            }
            if(gamepad2.x){
                stateMap.put(robot.LIFT_SYSTEM_NAME, robot.LIFT_SUBHEIGHT);
            }
            if(gamepad2.b){
                arm.extension.setPosition(0.75);
            }
            if(gamepad2.left_bumper){
                arm.grabber.setPosition(0.55);
            }
            if(gamepad2.right_bumper){
                arm.grabber.setPosition(0.75);
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

        }

        // operator controlled turret via right stick x axis
        /*
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

         */
        telemetry.addData("Lift" , lift.getPosition());
        //telemetry.addData("Turret", robot.turret.getPosition());
        //telemetry.addData("Extension", robot.arm.getExtensionPosition());

        telemetry.update();


    }

}
