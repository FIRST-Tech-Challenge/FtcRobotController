package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

@TeleOp(name = "pidExtender")
public class pidExtender extends LinearOpMode {

    // Run the initialize function
    RobotInitialize robot;

    int liftPitchPosition = 0;
    int liftExtenderPosition = 0;

    double p = 2.67;
    double i = 2.05;
    double d = 0;
    double f = 3.3;


    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize(this, false);

        robot.liftExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialization of the control of the robot when start is pressed
        waitForStart();
        while(opModeIsActive()) {
            // controller inputs that is inputted by the drive team
            controllerInput();
        }
    }

    public void controllerInput() {
        if ((Math.abs(gamepad2.right_stick_y)>0.2)
                &&(liftExtenderPosition<=900)
                &&(liftExtenderPosition>=0)
                ||(robot.liftExtender.getCurrentPosition()<0&&gamepad2.right_stick_y<0)
                ||(robot.liftExtender.getCurrentPosition()>900&&gamepad2.right_stick_y>0)) {

            liftExtenderPosition = liftExtenderPosition - (int)(10*gamepad2.right_stick_y);
            if(liftExtenderPosition < 0)
                liftExtenderPosition = 0;
            if(liftExtenderPosition > 900)
                liftExtenderPosition = 900;
        }
        //Determines if the liftExtender should go up or down based on the controller inputs
        if (liftExtenderPosition<=5&&robot.liftExtender.getCurrentPosition()<=5) {
            robot.liftExtender.setPower(0);
           // robot.liftExtender.setVelocity(0);
            telemetry.addLine("RESTING");

        }else if(Math.abs(robot.liftExtender.getCurrentPosition()-liftExtenderPosition)>5) {
            if (robot.liftExtender.getCurrentPosition() < liftExtenderPosition) {
                telemetry.addLine("MOVING UP");
                robot.liftExtender.setPower(.6);
                //robot.liftExtender.setVelocity(300);
            } else if (robot.liftExtender.getCurrentPosition() >= liftExtenderPosition) {
                robot.liftExtender.setPower(-.6);
               // robot.liftExtender.setVelocity(-300);
                telemetry.addLine("MOVING DOWN");

            }
            //If no input, make sure the liftExtender motor does not move
        }else {
            robot.liftExtender.setPower(0.01);
//            robot.liftExtender.setVelocity(0);
            telemetry.addLine("BRAKING");

        }

        if (gamepad2.right_trigger == 1){
            p=0;
            i=0;
            d=0;
            f=0;
        }

        if (gamepad2.dpad_up){
            p += 0.01;
        }if (gamepad2.dpad_down){
            i += 0.01;
        }if (gamepad2.dpad_right){
            d += 0.01;
        }if (gamepad2.dpad_left){
            f += 0.01;
        }

        if (gamepad2.triangle){
            p -= 0.01;
        }if (gamepad2.cross){
            i -= 0.01; //Avoid negative i value because it damages the motor
        }if (gamepad2.circle){
            d -= 0.01;
        }if (gamepad2.square){
            f -= 0.01;
        }


        //robot.liftExtender.setVelocityPIDFCoefficients(p, i, d, f);

        telemetry.addData("p", p);
        telemetry.addData("d", d);
        telemetry.addData("i", i);
        telemetry.addData("f", f);
        telemetry.addData("target pos", liftExtenderPosition);
        telemetry.addData("current pos", robot.liftExtender.getCurrentPosition());

        telemetry.update();
    }

}

