package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Jon DC")
public class JonDC extends LinearOpMode{
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    public void runOpMode(){
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        double[] power = new double[4];
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean ct = false;
        boolean prevInput = false;
        boolean ct2 = false;
        boolean prevInput2 = false;
        boolean lsDeadzone = false;
        boolean rsDeadzone = false;

        waitForStart();

        while (opModeIsActive()) {

            /*
            Deadzones | true = stick is near neutral position
            Protects from drift
            */
            lsDeadzone = (Math.abs(this.gamepad1.left_stick_x) < 0.1) &&
                         (Math.abs(this.gamepad1.left_stick_y) < 0.1);
            rsDeadzone = (Math.abs(this.gamepad1.right_stick_x) < 0.1) &&
                         (Math.abs(this.gamepad1.right_stick_y) < 0.1);

            //reset
            power[0]=0;
            power[1]=0;
            power[2]=0;
            power[3]=0;

            //translation
            if(!lsDeadzone){
                power[0]+=this.gamepad1.left_stick_y;
                power[1]+=this.gamepad1.left_stick_y;
                power[2]+=this.gamepad1.left_stick_y;
                power[3]+=this.gamepad1.left_stick_y;

                power[0]-=this.gamepad1.left_stick_x;
                power[1]+=this.gamepad1.left_stick_x;
                power[2]+=this.gamepad1.left_stick_x;
                power[3]-=this.gamepad1.left_stick_x;
            }

            //rotation
            if(!rsDeadzone){
                power[0]+=this.gamepad1.right_stick_x;
                power[1]-=this.gamepad1.right_stick_x;
                power[2]+=this.gamepad1.right_stick_x;
                power[3]-=this.gamepad1.right_stick_x;
            }

            //assigning speeds
            fl.setPower(power[0]); //(+) for 2020-21 Mayhem bot
            fr.setPower(power[1]); //(-) for 2020-21 Mayhem bot
            bl.setPower(power[2]); //(+) for 2020-21 Mayhem bot
            br.setPower(power[3]); //(-) for 2020-21 Mayhem bot


        }
    }

}



