package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class testlinearslide extends OpMode{

    private DcMotor lmotor;

   @Override
    public void init()
   {
       lmotor = hardwareMap.get(DcMotor.class, "linearSlide");
       lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
       lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       lmotor.setTargetPosition(0);


   }

   @Override
   public void loop()
    {
        int lmotorpos = lmotor.getCurrentPosition();
        if(gamepad1.right_trigger != 0)
        {
            lmotor.setTargetPosition(lmotorpos + 60);
            lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lmotor.setPower(0.8);
            telemetry.addData("Current position", lmotor.getCurrentPosition());

        }

        if(gamepad1.left_trigger != 0)
        {

            lmotor.setTargetPosition(lmotorpos - 60);
            lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lmotor.setPower(0.8);
            telemetry.addData("Current position", lmotor.getCurrentPosition());

        }

    }
}
