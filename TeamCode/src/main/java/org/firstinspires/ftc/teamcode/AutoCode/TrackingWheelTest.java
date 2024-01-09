package org.firstinspires.ftc.teamcode.AutoCode;

import android.util.LruCache;

//import com.qualcomm.hardware.lynx.LynxDcMotorController;
//import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;

import java.util.Base64;

@TeleOp
public class TrackingWheelTest extends LinearOpMode
{
    MovingStatistics movingStatistics = new MovingStatistics(300);

    long startLoop = 0;
    private DcMotorEx LeftTW;
    private DcMotorEx RightTW;
    private DcMotorEx BackTW;

    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

    @Override
    public void runOpMode() throws InterruptedException
    {
//        LynxModule module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 2");
//
//        LynxDcMotorController ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 2");
        //FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");

        LeftTW = hardwareMap.get(DcMotorEx.class, "LHang");
        RightTW = hardwareMap.get(DcMotorEx.class, "RHang");
        BackTW = hardwareMap.get(DcMotorEx.class, "intake");


//        ctrl.setMotorMode(0, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        ctrl.setMotorMode(0, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftTW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightTW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackTW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftTW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightTW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackTW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        startLoop = System.currentTimeMillis();
        double inch;
        double cmToInch;


        while (!this.isStopRequested() && this.isStarted())
        {
            startLoop = System.currentTimeMillis();

//            LynxModule.BulkData bulkData = module.getBulkData();

            //left = (int) (left * 1.02462166);
//            int left = -bulkData.getMotorCurrentPosition(2);
//            int right = -bulkData.getMotorCurrentPosition(1);
//            int aux = -bulkData.getMotorCurrentPosition(0);

            int left = -LeftTW.getCurrentPosition();
            int right = -RightTW.getCurrentPosition();
            int aux = -BackTW.getCurrentPosition();

            //cmToInch = FrontSonar.getDistanceSync();
            //inch = cmToInch/2.54;
            trackingWheelIntegrator.update(left, right, aux);
            //trackingWheelIntegratorArc.update(left, right, aux, telemetry);
            //telemetry.addData("FrontInch", inch);
            telemetry.addData("X", trackingWheelIntegrator.getX());
            telemetry.addData("Y", trackingWheelIntegrator.getY());
            //telemetry.addData("A_X", trackingWheelIntegratorArc.getX());
            //telemetry.addData("A_Y", trackingWheelIntegratorArc.getY());
            telemetry.addData("wheelH", trackingWheelIntegrator.getHeading());
            telemetry.addData("L", left*-1);
            telemetry.addData("R", right*-1);
            telemetry.addData("B", aux);
            telemetry.addData("S", trackingWheelIntegrator.speed());
            telemetry.addData("U", 1000/movingStatistics.getMean());

            telemetry.update();

            //positionLogger.add(trackingWheelIntegrator.getX(), trackingWheelIntegrator.getY());

            long delta = System.currentTimeMillis() - startLoop;
            movingStatistics.add(delta);
        }

        /*new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                positionLogger.dump();

                AppUtil.getInstance().showToast(UILocation.BOTH, "Dump done");
            }
        }).start();*/
    }
}
