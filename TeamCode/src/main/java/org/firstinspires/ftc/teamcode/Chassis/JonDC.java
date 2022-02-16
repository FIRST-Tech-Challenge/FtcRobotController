package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Arm.Arm;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Carousel.Carousel;

@TeleOp(name = "Jon DC")
public class JonDC extends LinearOpMode{
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DcMotor carouselTurningMotor;


    public void runOpMode(){
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        carouselTurningMotor = hardwareMap.dcMotor.get("carouselTurningMotor");


        //fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float[] power = new float[4];
        boolean lsDeadzone = false;
        boolean rsDeadzone = false;
        boolean direction1 = true;
        boolean direction2 = false;

        waitForStart();
        Carousel car = new Carousel(hardwareMap);
        Chassis ch = new Chassis(hardwareMap);
        Intake it = new Intake(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        while (opModeIsActive()) {
            //carousel code
            car.toggleDirection(this.gamepad2.b);
            car.toggleCarousel(this.gamepad2.a);
            it.intakeHold(this.gamepad2.x, this);
            //drive
            ch.slowMode(this.gamepad1.right_bumper);
            power = ch.mecanumDr(this.gamepad1.left_stick_x,this.gamepad1.left_stick_y,this.gamepad1.right_stick_x,this.gamepad1.right_stick_y);
            //assigning speeds
            fl.setPower(-power[0]); //(+) for 2020-21 Mayhem bot | (-) for 2021-22
            fr.setPower(power[1]); //(-) for 2020-21 Mayhem bot | (+) for 2021-22
            bl.setPower(-power[2]); //(+) for 2020-21 Mayhem bot | (-) for 2021-22
            br.setPower(power[3]); //(-) for 2020-21 Mayhem bot | (+) for 2021-22

            /*
            fl.setPower(-.5); //(+) for 2020-21 Mayhem bot | (-) for 2021-22
            fr.setPower(.5); //(-) for 2020-21 Mayhem bot | (+) for 2021-22
            bl.setPower(.5); //(+) for 2020-21 Mayhem bot | (-) for 2021-22
            br.setPower(-.5);*/
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



