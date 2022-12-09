package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Grnd_Junc_Sig_Prk101", group="Autonomous")
//@Disabled
public class Grnd_Junc_Sig_Prk101 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;  //left front wheel
    private DcMotor rf = null;  //right front wheel
    private DcMotor lb = null;  //left back wheel
    private DcMotor rb = null;  //right back wheel
    private DcMotor tower1 = null; //arm motor 1
    private DcMotor tower2 = null; //arm motor 2

    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        tower1 = hardwareMap.get(DcMotor.class, "tower1");

        double sidemult = -1.0; //Red side = 1.0 Blue = -1.0


        lf.setDirection(DcMotor.Direction.FORWARD);//do we need?
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);
        tower1.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 30.0)) {


           /*tower1.setPower(0);
            lf.setPower(-0.5);
            rf.setPower(-0.5);
            lb.setPower(0.5);
            rb.setPower(0.5);

            sleep(750); // strafe left


            tower1.setPower(0);
            lf.setPower(-0.5 * sidemult);
            rf.setPower(-0.5 * sidemult);
            lb.setPower(-0.5 * sidemult);
            rb.setPower(-0.5 * sidemult);

            sleep(500); // Forward to low junction

            tower1.setPower(-1);
            lf.setPower(0.5);
            rf.setPower(0.5);
            lb.setPower(0.5);
            rb.setPower(0.5);

            sleep(500); // Drop cone in low junction

            tower1.setPower(0);
            lf.setPower(0.5);
            rf.setPower(0.5);
            lb.setPower(0.5);
            rb.setPower(0.5);

            sleep(500); // Back away from low junction

            tower1.setPower(0);
            lf.setPower(0.5);
            rf.setPower(0.5);
            lb.setPower(-0.5);
            rb.setPower(-0.5);

            sleep(500); // Strafe right

            tower1.setPower(-1);
            lf.setPower(-0.5);
            rf.setPower(-0.5);
            lb.setPower(-0.5);
            rb.setPower(-0.5);

            sleep(500); // Forward and scan cone

            tower1.setPower(0);
            lf.setPower(0.5);
            rf.setPower(0.5);
            lb.setPower(0.5);
            rb.setPower(0.5);

            sleep(750); // Park in designated parking space */

            tower1.setPower(0);
            lf.setPower(0.5);
            rf.setPower(0.5);
            lb.setPower(0.5);
            rb.setPower(0.5);

            sleep(250); //park

            tower1.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);

            sleep(25000); // Stop
        }
    }
} 