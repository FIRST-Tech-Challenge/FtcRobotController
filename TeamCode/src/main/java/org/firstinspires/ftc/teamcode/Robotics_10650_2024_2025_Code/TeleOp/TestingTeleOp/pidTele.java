package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

@TeleOp(name = "pid")
public class pidTele extends LinearOpMode {

    // Run the initialize function
    RobotInitialize robot;

    int liftPitchPosition = 0;
    int liftExtenderPosition = 0;
    double p = 0;
    double i = 0;
    double d = 0;
    double f = 0;


    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize(this, false);


        // initialization of the control of the robot when start is pressed
        waitForStart();
        while(opModeIsActive()) {
            // controller inputs that is inputted by the drive team
            controllerInput();
        }
    }

    public void controllerInput() {
        if ((Math.abs(gamepad2.right_stick_y)>0.2)
                &&(liftExtenderPosition<=5000)
                &&(liftExtenderPosition>=0)
                ||(robot.liftExtender.getCurrentPosition()<0&&gamepad2.right_stick_y<0)
                ||(robot.liftExtender.getCurrentPosition()>5000&&gamepad2.right_stick_y>0)) {

            liftExtenderPosition = liftExtenderPosition - (int)(30*gamepad2.right_stick_y);
            if(liftExtenderPosition < 0)
                liftExtenderPosition = 0;
            if(liftExtenderPosition > 5000)
                liftExtenderPosition = 5000;
        }
        //Determines if the liftExtender should go up or down based on the controller inputs
        if (liftExtenderPosition<=5&&robot.liftExtender.getCurrentPosition()<=5) {
            robot.liftExtender.setVelocity(0);
        }else if(Math.abs(robot.liftExtender.getCurrentPosition()-liftExtenderPosition)>25) {
            if (robot.liftExtender.getCurrentPosition() < liftExtenderPosition) {
                robot.liftExtender.setVelocity(2000); //was 1500
            } else if (robot.liftExtender.getCurrentPosition() >= liftExtenderPosition) {
                robot.liftExtender.setVelocity(-2000); //was -1500
            }
            //If no input, make sure the liftExtender motor does not move
        }else {
            robot.liftExtender.setVelocity(1);
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
            i -= 0.01;
        }if (gamepad2.circle){
            d -= 0.01;
        }if (gamepad2.square){
            f -= 0.01;
        }


        robot.liftExtender.setVelocityPIDFCoefficients(p, i, d, f);

        telemetry.addData("p", p);
        telemetry.addData("d", d);
        telemetry.addData("i", i);
        telemetry.addData("f", f);
        telemetry.addData("target pos", liftExtenderPosition);
        telemetry.addData("current pos", robot.liftExtender.getCurrentPosition());

        telemetry.update();
    }

}

