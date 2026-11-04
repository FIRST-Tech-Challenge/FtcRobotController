package org.nknsd.teamcode.programs.tests.thisYear.aprilTag;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.DenseMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.nknsd.teamcode.components.handlers.odometry.CameraFlowFuser;
import org.nknsd.teamcode.components.sensors.AprilTagSensor;
import org.nknsd.teamcode.components.sensors.AprilTagSensor3d;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.utility.StateMachine;
import org.nknsd.teamcode.components.utility.StateMachine.State;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.frameworks.NKNProgram;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Camera Flow Tester", group="Tests")
public class CameraFlowTest extends NKNProgram {


    class giveGarbageData extends StateMachine.State {

    AprilTagSensor3d aprilSensor = new AprilTagSensor3d();
    FlowSensor flowSensor = new FlowSensor("");
    CameraFlowFuser flowFuser = new CameraFlowFuser(flowSensor, aprilSensor);

        float [] rot = new float[]{0, 0, 0};

        GeneralMatrixF rotationMatrix = new GeneralMatrixF(3, 3);

        int testNumber = 1;

        @Override
        protected void run(ElapsedTime runtime, Telemetry telemetry) {
                if (runtime.milliseconds() < 3000) {
                    if(testNumber == 1) {


                        rotationMatrix = flowFuser.findRotationMatrix(rot[0], rot[1], rot[2]);


                        telemetry.addData("Rotation 1: ", Arrays.toString(rot));
                        telemetry.addData("RotationMatrix: ", rotationMatrix);
                        RobotLog.v("Rotation 1 roll pitch yaw: " + rotationMatrix.toString());
                        RobotLog.v("Current rotat: " + Arrays.toString(rot));

                        testNumber++;
                    }
                }

                rot = new float[]{(float)(Math.PI/2), 0, 0};

                if (runtime.milliseconds() < 6000 && runtime.milliseconds() > 3000) {

                    if(testNumber == 2) {
                        rotationMatrix = flowFuser.findRotationMatrix(rot[0], rot[1], rot[2]);


                        telemetry.addData("Rotation 2 (roll = 1): ", rot);
                        telemetry.addData("RotationMatrix: ", rotationMatrix);
                        RobotLog.v("Rotation 2 roll pitch yaw: " + rotationMatrix.toString());
                        RobotLog.v("Current rotat: " + Arrays.toString(rot));

                        testNumber++;
                    }
                }

                rot = new float[]{0, 90, 0};
                if (runtime.milliseconds() < 9000 && runtime.milliseconds() > 6000) {

                    if(testNumber == 3) {

                        rotationMatrix = flowFuser.findRotationMatrix(rot[0], rot[1], rot[2]);


                        telemetry.addData("Rotation 3 (pitch = 1): ", rot);
                        telemetry.addData("RotationMatrix: ", rotationMatrix);
                        RobotLog.v("Rotation 3 roll pitch yaw: " + rotationMatrix.toString());
                        RobotLog.v("Current rotat: " + Arrays.toString(rot));

                        testNumber++;
                    }
                }

                rot = new float[]{0, 0, 1};
                if (runtime.milliseconds() < 12000 && runtime.milliseconds() > 9000) {
                    if(testNumber == 4) {

                        rotationMatrix = flowFuser.findRotationMatrix(rot[0], rot[1], rot[2]);


                        telemetry.addData("Rotation 4 (yaw = 1): ", rot);
                        telemetry.addData("RotationMatrix: ", rotationMatrix);
                        RobotLog.v("Rotation 4 roll pitch yaw: " + rotationMatrix.toString());
                        RobotLog.v("Current rotat: " + Arrays.toString(rot));

                        testNumber++;
                    }
                }

                rot = new float[]{1, 1, 1};
                if (runtime.milliseconds() < 15000 && runtime.milliseconds() > 18000) {
                    if(testNumber == 5) {

                        rotationMatrix = flowFuser.findRotationMatrix(rot[0], rot[1], rot[2]);


                        telemetry.addData("Rotation 5 (all 1): ", rot);
                        telemetry.addData("RotationMatrix: ", rotationMatrix);
                        RobotLog.v("Rotation 5 roll pitch yaw: " + rotationMatrix.toString());
                        RobotLog.v("Current rotat: " + Arrays.toString(rot));

                    }
                }
            }

        @Override
        protected void started() {

        }

        @Override
        protected void stopped() {

        }
    }

    @Override
    public void createComponents(List<NKNComponent> components, List<NKNComponent> telemetryEnabled) {

        components.add(StateMachine.INSTANCE);

        StateMachine.INSTANCE.startAnonymous(new giveGarbageData());

    }


}
