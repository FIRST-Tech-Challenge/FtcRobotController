package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.motorSlide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="LiamArmTest", group="B")
public class LiamArmTest extends DriveMethods {
    DcMotor motorScissor30;
    DcMotor motorScissor60;
    int i = 0;

    @Override
    public void runOpMode() {

        motorScissor30 = hardwareMap.get(DcMotor.class, "scissor30");
        motorScissor60 = hardwareMap.get(DcMotor.class, "scissor60");


        motorScissor60.setDirection(DcMotorSimple.Direction.REVERSE);

        motorScissor30.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorScissor60.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorScissor30.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorScissor60.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        goToScissorHeight(700);

        while (opModeIsActive()) {

        }

    }


    // !!!!!! NO MORE THAN 1100 Clicks !!!!!!
    public void goToScissorHeight(int Clicks) {
        /**
         * 60 rpm motor travels twice the distance and goes the opposite direction of the 30 rpm motor
         */
        //The "30" represents the 30 rpm base motor (first hinge)
        //The "60" represents the 60 rpm higher motor (second hinge)
        //Because they have different rpms by a factor of 2, 60rpm has half the clicks/rotation of the 30rpm


        int clicksPerRot30 = 5281;
        int clicksPerRot60 = 2786;
        int target30 = Clicks;
        int target60 = Clicks; //target60 despite having half the value, needs to travel twice the distance, so they are in fact the same value :)
        double current30 = motorScissor30.getCurrentPosition();
        double current60 = motorScissor60.getCurrentPosition();
        double error30 = target30 - current30; //error of the 30rpm motor
        double error60 = target60 - current60; //error of the 60rpm motor
        double hingeScaleFactor = 0.979123; //This is 469mm/479mm (the length different between each 'arm')
        double power30 = 0;
        double power60 = 0;
        double currentAngle30 = 0;
        double currentAngle60 = 0;

        telemetry.addLine("30Position: " + current30);
        telemetry.addLine("60Position: " + current60);
        telemetry.addLine("error30: " + error30);
        telemetry.addLine("error60: " + error60);
        telemetry.addLine("power30: " + power30);
        telemetry.addLine("power60: " + power60);
        telemetry.update();

        sleep(500);

        while(Math.abs(error30) >= 10 && Math.abs(error60) >= 10){
            current30 = motorScissor30.getCurrentPosition();
            current60 = motorScissor60.getCurrentPosition(); //This value's resolution is half the resolution of the above value

            error30 = target30 - current30;
            error60 = target60 - current60;

            currentAngle30 = (current30/clicksPerRot30)*360;
            currentAngle60 = (current60/clicksPerRot60)*360;

            power30 = 0.52*(1 - (currentAngle30/110) + error30/900);
            power60 = 0.43*(1- (currentAngle60/180) + error60/450);

            motorScissor30.setPower(power30);
            motorScissor60.setPower(power60); //the 60 rpm is traveling twice the distance of the 30 rpm at all times

            telemetry.addLine("30Position: " + motorScissor30.getCurrentPosition());
            telemetry.addLine("60Position: " + motorScissor60.getCurrentPosition());
            telemetry.addLine("error30: " + error30);
            telemetry.addLine("error60: " + error60);
            telemetry.addLine("power30: " + power30);
            telemetry.addLine("power60: " + power60);
            telemetry.addLine("currentAngle30: " + currentAngle30);
            telemetry.addLine("currentAngle60: " + currentAngle60);

            telemetry.update();

            if(currentAngle30 > 80 || currentAngle60 > 160) {
                motorScissor60.setPower(0);
                motorScissor30.setPower(0);
                break;
            }
            i++;
        }


        telemetry.addLine("You're outside the loop!");
        telemetry.addLine("power30: " + (power30-0.04));
        telemetry.addLine("power60: " + (power60-0.04));
        telemetry.update();
        motorScissor30.setPower(power30 - 0.04);
        motorScissor60.setPower(power60 - 0.04);


    }
}
//hi