package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.mason;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ENCODER Test", group = "Encoder")

public class Test extends LinearOpMode {
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            double LFDis = LF.getCurrentPosition();
            double RFDis = RF.getCurrentPosition();
            double LBDis = LB.getCurrentPosition();
            double RBDis = RB.getCurrentPosition();
            telemetry.addData("LF Encoder Value: ", LFDis);
            telemetry.addData("RF Encoder Value: ", RFDis);
            telemetry.addData("LB Encoder Value: ", LBDis);
            telemetry.addData("RB Encoder Value: ", RBDis);
            telemetry.update();
        }
    }



}
