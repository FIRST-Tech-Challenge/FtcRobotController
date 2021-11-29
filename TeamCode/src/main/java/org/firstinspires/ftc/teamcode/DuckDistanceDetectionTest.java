package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="distance Detect")
@Disabled
public class DuckDistanceDetectionTest extends LinearOpMode {
    zanehardware robot = new zanehardware();
    private ElapsedTime runtime = new ElapsedTime();

    public static boolean seenObject = false;


    @Override
    public void runOpMode() throws InterruptedException {
     waitForStart();
     int location = barcodeDetect();

    }



    public int barcodeDetect(){
        int count = 0;
        int Location = 0;
        for (int i = 0;  i < 4; i++ ){
            count ++;
            if (robot.LeftDistance.getDistance(DistanceUnit.INCH) <= 18 && seenObject == false){
                Location = count;
                seenObject = true;
                encoderDrive(0.5,8,-8,8,-8,3);
            }else if(seenObject == true){
                break;
            }else{

            }
        }

        return Location;
    }
    public void encoderDrive(double speed,
                             double Back_Left_Inches,
                             double Back_Right_Inches,
                             double Front_Right_Inches,
                             double Front_Left_Inches,
                             double timeoutS) {
        int newLeftBottomTarget;
        int newRightBottomTarget;
        int newRightTopTarget;
        int newLeftTopTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            double COUNTS_PER_INCH = 122.600924;
            newLeftBottomTarget = robot.Back_Left.getCurrentPosition() + (int)(Back_Left_Inches * COUNTS_PER_INCH);
            newRightBottomTarget = robot.Back_Right.getCurrentPosition() + (int)(Back_Right_Inches * COUNTS_PER_INCH);
            newRightTopTarget = robot.Front_Right.getCurrentPosition() + (int) (Front_Right_Inches * COUNTS_PER_INCH);
            newLeftTopTarget = robot.Front_Left.getCurrentPosition() + (int) (Front_Left_Inches * COUNTS_PER_INCH);

            robot.Back_Left.setTargetPosition(newLeftBottomTarget);
            robot.Back_Right.setTargetPosition(newRightBottomTarget);
            robot.Front_Right.setTargetPosition(newRightTopTarget);
            robot.Front_Left.setTargetPosition(newLeftTopTarget);

            // Turn On RUN_TO_POSITION
            robot.Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.Back_Left.setPower(Math.abs(speed));
            robot.Back_Right.setPower(Math.abs(speed));
            robot.Front_Left.setPower(Math.abs(speed));
            robot.Front_Right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.Back_Left.isBusy() && robot.Back_Right.isBusy() && robot.Front_Left.isBusy() && robot.Front_Right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBottomTarget,newRightBottomTarget,newLeftTopTarget,newRightTopTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.Back_Left.getCurrentPosition(),
                        robot.Back_Right.getCurrentPosition());
                robot.Front_Left.getCurrentPosition();
                robot.Front_Right.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            robot.Back_Left.setPower(0);
            robot.Back_Right.setPower(0);
            robot.Front_Left.setPower(0);
            robot.Front_Right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }
}
