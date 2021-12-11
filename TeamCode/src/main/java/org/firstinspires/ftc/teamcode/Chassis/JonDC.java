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

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float[] power = new float[4];
        boolean lsDeadzone = false;
        boolean rsDeadzone = false;

        waitForStart();

        while (opModeIsActive()) {

            Chassis ch = new Chassis(hardwareMap);

            power = ch.mecanumDr(this.gamepad1.left_stick_x,this.gamepad1.left_stick_y,this.gamepad1.right_stick_x,this.gamepad1.right_stick_y);
            //assigning speeds
            fl.setPower(-power[0]); //(+) for 2020-21 Mayhem bot | (-) for 2021-22
            fr.setPower(power[1]); //(-) for 2020-21 Mayhem bot | (+) for 2021-22
            bl.setPower(-power[2]); //(+) for 2020-21 Mayhem bot | (-) for 2021-22
            br.setPower(power[3]); //(-) for 2020-21 Mayhem bot | (+) for 2021-22

            /* --------------------------------------
            Telemetry Stuff
            -------------------------------------- */

            telemetry.addData("Front Left Motor",power[0]);
            telemetry.addData("Front Right Motor",power[1]);
            telemetry.addData("Back Left Motor",power[2]);
            telemetry.addData("Back Right Motor",power[3]);
            //display direction
            String[] dirDisp = dirDisplay(this.gamepad1.left_stick_x,this.gamepad1.left_stick_y);
            for (String dirLine: dirDisp){
                telemetry.addLine(dirLine);
            }
            telemetry.update();
        }
    }

    //Displays a Tic Tac Toe board of directions (may need adjustment depending on bot)
    public static String[] dirDisplay(float xVal, float yVal){
        String[] res = new String[3];
        String[][] l1 = {{"O","|","O","|","O"}, {"O","|","O","|","O"}, {"O","|","O","|","O"}};
        String temp = "";

        int ylvl = 1;
        if (yVal>.2) {
            ylvl = 2;
        } else if (yVal<-.2) {
            ylvl = 0;
        }

        int xlvl = 2;
        if (xVal>.2) {
            xlvl = 4;
        } else if (xVal<-.2) {
            xlvl = 0;
        }

        l1[ylvl][xlvl] = "X";

        for(int i = 0;i<3;i++){
            temp = "";
            for (int ii = 0; ii<5; ii++){
                temp = (temp + l1[i][ii]);
            }
            res[i] = temp;
        }

        return res;
    }
}



