package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class encoder_auto extends LinearOpMode {


    private DcMotor crane;

    public void runOpMode() {
        crane = hardwareMap.get(DcMotor.class, "crane");

        waitForStart();

        if (opModeIsActive()) {

            //crane(1,1000);
            crane.setTargetPosition(100);
            crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            crane.setPower(1);
            sleep(8000);


            telemetry.addData("encoder value", crane.getCurrentPosition());
            telemetry.update();
        }
    }


    public void crane(double power,int time){
        crane.setPower(power);
        sleep(time);
        crane.setPower(0);
    }
}


