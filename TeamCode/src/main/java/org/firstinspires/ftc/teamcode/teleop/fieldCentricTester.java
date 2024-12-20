package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class fieldCentricTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        //    if(controller1.y.onPress())
//            {
//                fieldCentric = !fieldCentric;
//            }
//                if(fieldCentric)
//                {
//                    telemetry.addData("Field Centric: ", fieldCentric);
//                    double forward = -controller1.left_stick_y;
//                    double strafe = controller1.left_stick_x;
//                    double rotate = -controller1.right_stick_x;
//                    clawPos = 0.6;
//                    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//
//                    //drive and limelight
//                    String[] distance =limelight.getDistanceInInches();
//                    telemetry.addData("Limelight Distance: ", distance[0] + ", " + distance[1]);
//                    drive.updatePoseEstimate();
//                    robot.fieldCentricDrive(forward, strafe, rotate);
//                    telemetry.addData("x", screen.roundData(drive.pose.position.x));
//                    telemetry.addData("y", screen.roundData(drive.pose.position.y));
//                    telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));
//
//
//
//                    //claw
//                    if (controller1.a.onPress()) {
//                        clawIn = !clawIn;
//                    }
//                    if(clawIn == true)
//                    {
//                        claw.close(clawPos);
//                    }else{
//                        claw.release();
//                    }
//                    telemetry.addData("Claw Position", clawPos);
//
//
//                    //lift
//
//    //                liftPower = controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue();
//                    if(controller1.left_trigger.onPress())
//                    {
//                        liftPos = liftPositions[liftPositionIndex];
//
//                        liftPositionIndex++;
//
//                        if(liftPositionIndex > liftPositions.length)
//                        {
//                            liftPositionIndex = 0;
//                        }
//                    }
//                    lift.setPosition(liftPos);
//                    lift.moveLift(liftPower);
//                    telemetry.addData("Left lift pos", lift.getPosition());
//                    telemetry.addData("Right lift pos", lift.getRightPosition());
//                    telemetry.update();
//                    controller1.update();
//                }
    }


}
