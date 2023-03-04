package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ManualControl {
    static void control(String mode, Project1Hardware robot, Telemetry telemetry, Gamepad gamepad1){
        double bucketSPos = robot.bucket.getPosition();
        double bucketPos = robot.bucketAngle.getPosition();
        if (mode == "VERT"){
            if (gamepad1.left_bumper) {
                robot.vert.setPower(0.5);
            }
            else if (gamepad1.right_bumper){
                robot.vert.setPower(-0.3);
            }
            else{
                robot.vert.setPower(0);
            }
            telemetry.addData("vertical position", robot.vert.getCurrentPosition());
        }
        else if (mode == "HORZ"){
            if (gamepad1.left_bumper) {
                robot.horz.setPower(-0.8);
            }
            else if (gamepad1.right_bumper){
                robot.vert.setPower(0.3);
            }
            else{
                robot.vert.setPower(0);
            }
            telemetry.addData("horizontal position", robot.horz.getCurrentPosition());
        }
        else if (mode == "BUCKET"){
            if (gamepad1.left_bumper){
                robot.bucket.setPosition(bucketSPos + 0.01);
                bucketSPos += 0.01;
            }
            else if (gamepad1.right_bumper){
                robot.bucket.setPosition(bucketSPos - 0.01);
                bucketSPos -= 0.01;
            }
            telemetry.addData("bucket small position", bucketSPos);
            //1 (hold cup), 0.7
        }
        else if (mode == "BUCKETANGLE"){
            if (gamepad1.left_bumper){
                robot.bucketAngle.setPosition(bucketPos + 0.01);
                bucketPos += 0.01;
            }
            else if (gamepad1.right_bumper){
                robot.bucketAngle.setPosition(bucketPos - 0.01);
                bucketPos -= 0.01;
            }
            telemetry.addData("bucket position", bucketPos);
            //0.72 (drop bucket), 0.02
        }
    }
}
