
package org.firstinspires.ftc.teamcode;

import android.text.ParcelableSpan;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="tommy")

public class Project1 extends LinearOpMode
{
    final static double ARM_HOME = 0.0;
    final static double ARM_MIN_RANGE = 0.0;
    final static double ARM_MAX_RANGE = 1.0;
    final static double HEXMOTOR_TPR = 288;
    final static double GOBILDA_TPR = 751.8; //5202
    final static double GEAR_REDUCTION = 1;
    final static double TICKS_PER_DEGREE = (HEXMOTOR_TPR * GEAR_REDUCTION) / 360;
    //ticks = pulses, cycles = ticks * 4

    final String[] debugModes =  {"VERT", "HORZ", "BUCKET", "BUCKET ANGLE", "CLAW", "CLAW ANGLE"};
    int dModesIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.imu.resetYaw();

        //variables
        double direction_y, direction_x, pivot, heading;
        boolean clawOpen = true;
        double toggleTimer = 0, toggleOrig1 = 0, toggleInterval = 200;

        double pidTime = 0, pidTime1 = 0, pidTarget = 270;
        double Kp = 40, Ki = 0.002, Kd = 40;
        double cur, error = 0, lastError;
        double I = 0, D = 0;
        waitForStart();
        drivetrain.remote(0,0,0,0);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            ((DcMotorEx) robot.flip1).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0.002, 40, 20));
            ((DcMotorEx) robot.flip2).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0.002, 40, 20));
            direction_y   = -gamepad1.left_stick_y*0.7;
            direction_x = gamepad1.left_stick_x*0.7;
            pivot    =  gamepad1.right_stick_x*0.7;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // INTAKE
            /*
            if (gamepad1.square){
                Intake.lift("OFF", robot);
            }
            else if (gamepad1.cross){
                Intake.lift("LOW", robot);
            }
            else if (gamepad1.circle){
                Intake.lift("MID", robot);
            }
            else if (gamepad1.triangle){
                Intake.lift("HIGH", robot);
            }*/

            toggleTimer = timer.milliseconds();
            if ((toggleTimer-toggleOrig1) > toggleInterval) {
                if (gamepad1.right_bumper && !clawOpen){
                    clawOpen = true;
                    robot.claw1.setPosition(0);
                    robot.claw2.setPosition(-1);
                }
                else if(gamepad1.left_bumper && clawOpen){
                    clawOpen = false;
                    robot.claw1.setPosition(0.5);
                    robot.claw2.setPosition(-0.5);
                }
                toggleOrig1 = toggleTimer;
            }


            // DRIVETRAIN
            if (gamepad1.left_bumper) {  //Brakes/slow down car
                direction_x = direction_x/2;
                direction_y = direction_y/2;
                pivot = pivot/2;
            } // brake
            if ((gamepad1.left_trigger == 0) || (gamepad1.right_trigger == 0)){
                drivetrain.remote(direction_y, direction_x, pivot, heading);
            } // normal drive
            else {
                drivetrain.remote(0, ((gamepad1.right_trigger- gamepad1.left_trigger)*0.7), pivot, 0);
            } // horizontal strafe

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) robot.imu.resetYaw();

            if (gamepad1.right_bumper) {
                robot.vert.setTargetPosition(robot.vert.getCurrentPosition()+10);
                robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vert.setPower(1);
            }
            if (gamepad1.dpad_up){
                robot.flip1.setTargetPosition(100);
                robot.flip1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.flip1.setPower(0.8);

                //direction_x = direction_x/2

            }
            if (gamepad1.dpad_down){
                robot.flip2.setTargetPosition(-100);
                robot.flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.flip2.setPower(-0.8);
            }

            if (gamepad1.square){
                while (Math.abs(robot.flip1.getCurrentPosition()-pidTarget) <= 10){
                    if (gamepad1.circle){
                        pidTarget += 10;
                    }
                    cur = robot.flip1.getCurrentPosition();
                    pidTime1 = timer.milliseconds();
                    lastError = error;
                    error = pidTarget-cur;
                    I += error * (pidTime1-pidTime);
                    D = (error - lastError) / (pidTime1-pidTime);

                    double out = (error * Kp) + (I * Ki) + (D * Kd);

                    ((DcMotorEx) robot.flip1).setVelocity(out);

                    pidTime = pidTime1;

                    //telemetry.addData("Encoder value", robot.flip1.getCurrentPosition());
                    //telemetry.update();
                }
            }
            /*
            if (gamepad1.square) robot.claw1.setPosition(0);
            if (gamepad1.cross) robot.claw2.setPosition(0);
            if (gamepad1.triangle){robot.yaw1.setPosition(0);
            if (gamepad1.circle) robot.yaw1.setPosition(0);
            */

            if (gamepad1.triangle){
                Intake.lift ("HIGH", robot);
                robot.flip1.setTargetPosition(270);
                robot.flip1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.flip1.setPower(0.8);
                robot.flip2.setTargetPosition(-270);
                robot.flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.flip2.setPower(-0.8);
            }
            // TELEMETRY
            telemetry.addData("vert", robot.vert.getCurrentPosition());
            telemetry.update();

        }
    }
}