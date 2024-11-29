package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.purepursuit.LineSegment;
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuit;

@Autonomous(name="AutonomousPurePursuit", group="Linear Opmode2")
public class AutonomousPurePursuit extends CommonUtil {

    Orientation myRobotOrientation;

    PurePursuit autoPurePursuit = new PurePursuit(0., 0.);

    @Override
    public void runOpMode() {

        //setup

        telemetry.setAutoClear(false);
        // initialize hardware
        initialize(hardwareMap);
        // Initialize motors
        setMotorOrientation();
        double ENC2DIST = 200/29;
        setMotorToZeroPower();
        setZeroPowerBehavior();
        imu.resetYaw();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        autoPurePursuit.addSegmentToPath(new LineSegment(0.0, 0.0, 0.0, 40.0));
        telemetry.addData("Angle of Segment", autoPurePursuit.calculateMovementAngle());
        sleep(3000);
        telemetry.update();
        movePurePursuit(100,0.3,100);
        setMotorToZeroPower();
        autoPurePursuit.addSegmentToPath(new LineSegment(0.0, 40.0, -2.0, 10.0));
        telemetry.addData("Angle of Segment", autoPurePursuit.calculateMovementAngle());

        sleep(3000);
        telemetry.update();
        movePurePursuit(100,0.3,100);
        setMotorToZeroPower();
        autoPurePursuit.addSegmentToPath(new LineSegment(-40.0, 40.0, -40.0, 0.0));
        telemetry.addData("Angle of Segment", autoPurePursuit.calculateMovementAngle());
        sleep(3000);
        telemetry.update();
        movePurePursuit(100,0.3,100);
        setMotorToZeroPower();



        //7:1 2


    }


    public int movePurePursuit(double DistanceEncoders,double Mpower,int timeToStop)
    {

        ElapsedTime runtime= new ElapsedTime();
        double currZAngle = 0;
        double prevZAngle = 0;
        int currEncoderCount = 0;
        int prevEncoderCount = 0;
        double currErrEC = 0;
        double encoderAbsCounts = DistanceEncoders;
        telemetry.addData("EC Target", encoderAbsCounts);
        telemetry.update();

        // Resetting encoder counts
        resetMotorEncoderCounts();
        runtime.reset();
        double power2 = 0;

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double bl_pos = bl.getCurrentPosition();
        double fl_pos = fl.getCurrentPosition();
        double br_pos = br.getCurrentPosition();
        double fr_pos = fr.getCurrentPosition();
        double min_pos = Math.min(Math.min(br_pos,bl_pos),Math.min(fl_pos,fr_pos));

        // move forward
        while ((min_pos < encoderAbsCounts) && (runtime.seconds() < timeToStop)) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            currZAngle = myRobotOrientation.thirdAngle;
            double correction = PID_Turn(autoPurePursuit.calculateMovementAngle(), currZAngle, "off");
            currEncoderCount = (int)(min_pos);

            double power = Mpower;
            int moveTime = (int) ((10 * Mpower) * ((double) 50 / 3));
            int movePause = (int) ((10 * Mpower) * ((double) 10 / 3));
            telemetry.addData("moveTime", moveTime);
            telemetry.addData("movePause", movePause);
            telemetry.update();

            bl.setPower(power + correction);
            fl.setPower(power + correction);
            fr.setPower(power - correction);
            br.setPower(power - correction);
            telemetry.addData("correctionvalue", correction);
       //     telemetry.addData("fw:power", power);
            telemetry.addData("curr pos", min_pos);
            telemetry.addData("encoderAbsCounts", encoderAbsCounts);
            telemetry.addData("targetAngle",autoPurePursuit.calculateMovementAngle());
            telemetry.update();

            bl_pos = bl.getCurrentPosition();
            fl_pos = fl.getCurrentPosition();
            br_pos = br.getCurrentPosition();
            fr_pos = fr.getCurrentPosition();
            min_pos = Math.min(Math.min(br_pos,bl_pos),Math.min(fl_pos,fr_pos));
        }


        telemetry.addData("curr pos", min_pos);
        telemetry.addData("encoderAbsCounts", encoderAbsCounts);

        telemetry.update();
        // apply zero power to avoid continuous power to the wheels
        // return current encoder count
        //currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        imu.resetYaw();
        return (int)(min_pos);
    }


}


