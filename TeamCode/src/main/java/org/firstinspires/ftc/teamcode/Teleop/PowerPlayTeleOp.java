package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.GBrobot;

@TeleOp
public class PowerPlayTeleOp extends OpMode {
    public GBrobot robot;

    @Override
    public void init() {
        this.robot = new GBrobot(this);
        robot.Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double slowFactor = 1;

        // Claw controls
        if(gamepad2.a){
            robot.claw.setClawPosition(0.25);
        }

        if(gamepad2.x){
            robot.claw.setClawPosition(0);
        }

        if (gamepad1.right_bumper) {
            slowFactor = .5;
        }

        // Drivetrain
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(drive)+Math.abs(strafe)+Math.abs(turn),1);

        double fl = ((drive+strafe+turn)/denominator)*slowFactor;
        double bl = ((drive-strafe+turn)/denominator)*slowFactor;
        double fr = ((drive-strafe-turn)/denominator)*slowFactor;
        double br = ((drive+strafe-turn)/denominator)*slowFactor;
        robot.Drive.setMotorPower(fl, bl, fr, br);

        // DR4B lift
        double lift = gamepad2.left_stick_y*0.8;
        robot.lift.SetMotorPower(lift);

        if(gamepad2.a){
            robot.lift.driveLiftToPosition(0.5, 500);
        }
        if (gamepad2.b){
            robot.lift.driveLiftToPosition(0.5, 1600);
        }
        if (gamepad2.y){
            robot.lift.driveLiftToPosition(0.5, 2800);
        }
        if (gamepad2.x){
            robot.lift.driveLiftToPosition(0.5, 3900);
        }
        if (gamepad2.back){
            if(robot.limit1.isPressed()){
                telemetry.addData("IS PRESSED:" , " true");
                robot.lift.SetMotorPower(0);
            }
            else{
                telemetry.addData("IS PRESSED", "false");
                robot.lift.SetMotorPower(-1);
            }
            telemetry.update();
        }

    }
}
