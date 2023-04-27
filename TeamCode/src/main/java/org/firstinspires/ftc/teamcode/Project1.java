
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        robot.imu1.resetYaw();

        //variables
        double direction_y, direction_x, pivot, heading;
        boolean clawOpen = true;
        double toggleTimer = 0, toggleOrig1 = 0, toggleInterval = 50;
        int vertPos;
        String mode = "OFF";
        robot.vert.setTargetPosition(0);
        waitForStart();
        drivetrain.remote(0,0,0,0);
        //robot.flip1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.vert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //((DcMotorEx) robot.flip1).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        //robot.flip1.setDirection(DcMotorEx.Direction.REVERSE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            direction_y   = gamepad1.left_stick_y *0.6;
            direction_x = -gamepad1.left_stick_x *0.6;
            pivot    =  gamepad1.right_stick_x * 0.7;
            heading = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            toggleTimer +=1 ;
            vertPos = robot.vert.getCurrentPosition();
            /*
            //if (gamepad1.square){
                robot.flip1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               // robot.flip2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //robot.flip1.setDirection(DcMotor.Direction.REVERSE);
                //while (Math.abs(robot.flip1.getCurrentPosition()-pidTarget) <= 10){ // CHANGE WHILE LOOP TO IF STATEMENT after tuning the values to keep the motor running
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

                    ((DcMotorEx) robot.flip1).setPower(out);


                    pidTime = pidTime1;

                    telemetry.addData("Encoder value", robot.flip1.getCurrentPosition());
                    //debug if u want it la
               // }
           // }
           */



            // INTAKE

            if (gamepad1.square){
                robot.vert.setTargetPosition(0);
            }
            else if (gamepad1.cross){
                robot.vert.setTargetPosition(650);
            }
            else if (gamepad1.circle){
                robot.vert.setTargetPosition(1300);
            }
            else if (gamepad1.triangle){
                robot.vert.setTargetPosition(2400);
            }
            /*else if (gamepad1.dpad_up){
                robot.vert.setTargetPosition(vertPos+20);
            }
            else if (gamepad1.dpad_down){
                robot.vert.setTargetPosition(vertPos-20);
            }*/

            if (gamepad1.dpad_up){
                drivetrain.part1(90, 0, 0.6);
            }
            else if (gamepad1.dpad_down){
                drivetrain.part1(270, 0, 0.6);
            }
            else if (gamepad1.left_stick_button){
                robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("LF", robot.frontLeft.getCurrentPosition());
            telemetry.addData("RF", robot.frontRight.getCurrentPosition());
            telemetry.addData("BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR", robot.backRight.getCurrentPosition());

            if ((toggleTimer-toggleOrig1) > toggleInterval) {
                if (gamepad1.right_bumper ){
                    clawOpen = (!clawOpen);
                    toggleTimer = toggleOrig1;
                }

            }
            if (clawOpen){
                robot.claw1.setPosition(0.8);
                robot.claw2.setPosition(1);
                robot.yaw1.setPosition(0.4);
                robot.yaw2.setPosition(0.3);
                robot.vert.setTargetPosition(0);

            }
            else{
                robot.claw1.setPosition(0.95);
                robot.claw2.setPosition(0.75);
                if (robot.vert.getCurrentPosition()>300){
                    if(gamepad1.left_bumper){
                        robot.yaw1.setPosition(0.4);
                        robot.yaw2.setPosition(0.3);
                    }
                    else{
                        robot.yaw1.setPosition(0.65);
                        robot.yaw2.setPosition(0.05);
                    }
                }
                else {
                    if (robot.vert.getCurrentPosition()<100){
                        sleep(300);
                        robot.vert.setTargetPosition(180);
                    }
                    else {
                        robot.yaw1.setPosition(0.4);
                        robot.yaw2.setPosition(0.3);
                    }

                }

            }


            //DRIVETRAIN
            if (robot.vert.getCurrentPosition()>800) {  //Brakes/slow down car
                direction_x = direction_x/2;
                direction_y = direction_y/2;
                pivot = pivot/2;
            } // brake
            if ((gamepad1.left_trigger == 0) && (gamepad1.right_trigger == 0)){
                drivetrain.remote(direction_y, direction_x, pivot, heading);
            } // normal drive
            else {
                drivetrain.remote(0, ((-gamepad1.right_trigger+ gamepad1.left_trigger)*0.3), pivot, 0);
            } // horizontal strafe

            if (gamepad1.touchpad) {
                robot.imu1.resetYaw();
            }

            //if (gamepad1.square || gamepad1.circle || gamepad1.triangle || gamepad1.cross){
                /*switch (mode) {
                    case "OFF": robot.vert.setTargetPosition(0);
                    case "LOW": robot.vert.setTargetPosition(500);
                    case "MID": robot.vert.setTargetPosition(800);
                    case "HIGH": robot.vert.setTargetPosition(1300);
                }*/

            //}
            robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vert.setPower(0.8);


            // TELEMETRY
            telemetry.addData("vert", robot.vert.getCurrentPosition());
            telemetry.addData("time", toggleTimer);
            telemetry.addData("yaw1", robot.yaw1.getPosition());
            telemetry.addData("yaw2", robot.yaw2.getPosition());
            telemetry.update();

            //while(gamepad1.left_stick_button){robot.yaw1.setPosition(-0.7);}

        }
    }
}