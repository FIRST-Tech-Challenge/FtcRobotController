package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.gary;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "GTeleOp_Intake_Spin", group = "Linear OpMode")
@Disabled
public class TeleOp_Intake_Spin extends LinearOpMode {
    private DcMotor Intake = null;
    private DcMotor Spin = null;


    @Override
    public void runOpMode(){
        Intake  = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

        Intake.setDirection(DcMotor.Direction.REVERSE); //anticlockwise
        Spin.setDirection(DcMotor.Direction.FORWARD); //clockwise
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double B_P = 1;
        double IntakeDirection = gamepad1.left_stick_y;
        double SpinDirection = gamepad1.right_stick_y;

        boolean Spin1 = true;
        boolean Intake1 = true;



        waitForStart();

        while(opModeIsActive()){
            double intakePower; //setting initial power
            double spinPower; //setting initial power

            //Spin
            if(gamepad1.a){
                if(Spin1){
                    spinPower = B_P;
                    telemetry.addLine("SPIN STARTS");
                    Spin1 = false;
                }
            }else if(!Spin1){
                Spin1 = true;
            }

            if(gamepad1.b){
                if(Spin1){
                    spinPower = -B_P;
                    telemetry.addLine("SPIN STARTS");
                    Spin1 = false;
                }
            }else if(!Spin1){
                Spin1 = true;
            }

            //intake
            if(gamepad1.x){
                if(Intake1){
                    intakePower = B_P;
                    telemetry.addLine("INTAKE STARTS");
                    Intake1 = false;
                }
            }else if(!Intake1){
                Intake1 = true;
            }

            if(gamepad1.y){
                if(Intake1){
                    intakePower = -B_P;
                    telemetry.addLine("INTAKE STARTS");
                    Intake1 = false;
                }
            }else if(!Intake1){
                Intake1 = true;
            }



            telemetry.addData("The power: ", B_P);
            telemetry.update();




        }

    }
}

/*
if(gamepad1.left_bumper == true){
                while(gamepad1.left_bumper){
                }
                B_P = B_P+0.1;
            }else if(gamepad1.right_bumper == true){
                while(gamepad1.right_bumper){
                }
                B_P = B_P-0.1;
            }

            if(gamepad1.a){
                spinPower = 1;
            }else if(gamepad1.b){
                intakePower = 1;
            }




            intakePower = Range.clip(B_P * (IntakeDirection), -1.0, 1.0);
            spinPower = Range.clip(B_P * (SpinDirection), -1.0, 1.0);
                       Intake.setPower(intakePower);
            Spin.setPower(spinPower);

 */
