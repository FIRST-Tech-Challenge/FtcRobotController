package org.nknsd.teamcode.components.handlers.odometry;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.matrices.ColumnMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.DenseMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.RowMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.nknsd.teamcode.components.sensors.AprilTagPosition;
import org.nknsd.teamcode.components.sensors.AprilTagSensor3d;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.frameworks.NKNComponent;

import java.util.Arrays;

public class CameraFlowFuser implements NKNComponent {

    private final static int SAMPLE_TIME_MS = 25;


    private final SparkFunOTOS.Pose2D Pos = new SparkFunOTOS.Pose2D(0, 0, 0);


    FlowSensor flowSensor;
    AprilTagSensor3d aprilSensor;

    //the tag position arrays are simply for easier access
    //they are set every loop, the values at 0 are closer to the center of the field.
    //Back refers to facing the audience, front refers to facing away from the audience.
    AprilTagPosition[] backBlue = new AprilTagPosition[4];
    AprilTagPosition[] frontBlue = new AprilTagPosition[4];
    AprilTagPosition[] backRed = new AprilTagPosition[4];
    AprilTagPosition[] frontRed = new AprilTagPosition[4];

    public CameraFlowFuser(FlowSensor flowSensor, AprilTagSensor3d aprilSensor) {
        this.flowSensor = flowSensor;
        this.aprilSensor = aprilSensor;
    }


    public GeneralMatrixF findRotationMatrix(float roll, float pitch, float yaw){
        GeneralMatrixF rotationMatrix = new GeneralMatrixF(3, 3);

        double rollSin = Math.sin(roll);
        double rollCos = Math.cos(roll);
        double pitchSin = Math.sin(pitch);
        double pitchCos = Math.cos(pitch);
        double yawSin = Math.sin(yaw);
        double yawCos = Math.cos(yaw);

        rotationMatrix.put(0, 0, (float) (rollCos * pitchCos));
        rotationMatrix.put(0, 1, (float) ((rollCos * pitchSin * yawSin) - (rollSin * yawCos)));
        rotationMatrix.put(0, 2, (float) ((rollCos * pitchSin * yawCos) + (rollSin * yawSin)));
        rotationMatrix.put(1, 0, (float) (rollSin * pitchCos));
        rotationMatrix.put(1, 1, (float) ((rollSin * pitchSin * yawSin) + (rollCos + yawCos)));
        rotationMatrix.put(1, 2, (float) ((rollSin * pitchSin * yawCos) + (rollCos * yawSin)));
        rotationMatrix.put(2, 0, (float) (-pitchSin));
        rotationMatrix.put(2, 1, (float) (pitchCos * yawSin));
        rotationMatrix.put(2, 2, (float) (pitchCos * yawCos));


        return rotationMatrix;
    }

    //  averages both sensors
    public SparkFunOTOS.Pose2D getPosition() {
        return Pos;
    }

    public double[] getDoublePosition(){
        double[] positions = new double[3];
        positions[0] = Pos.x;
        positions[1] = Pos.y;
        positions[2] = Pos.h;
        return positions;


    }

    //should be called at the start of every loop or elsewhere if needed.
    public void getAprilTags(){
        frontRed[0] = aprilSensor.getTag30();
        frontRed[1] = aprilSensor.getTag31();
        frontRed[2] = aprilSensor.getTag32();
        frontRed[3] = aprilSensor.getTag33();
        backRed[3] = aprilSensor.getTag34();
        backRed[2] = aprilSensor.getTag35();
        backRed[1] = aprilSensor.getTag36();
        backRed[0] = aprilSensor.getTag37();
        backBlue[0] = aprilSensor.getTag38();
        backBlue[1] = aprilSensor.getTag39();
        backBlue[2] = aprilSensor.getTag40();
        backBlue[3] = aprilSensor.getTag41();
        frontBlue[3] = aprilSensor.getTag42();
        frontBlue[2] = aprilSensor.getTag43();
        frontBlue[1] = aprilSensor.getTag44();
        frontBlue[0] = aprilSensor.getTag45();
    }




    private boolean hiveFacingUp(AprilTagPosition[] aprilTagPositions){
        //if the z value of a tag is above this, the tag is considered valid.
        int zMin = 30;
        //tracks number of "valid" tags for the hive to be facing up.
        int validTags = 0;

        for (AprilTagPosition aprilTagPosition : aprilTagPositions) {
            if (aprilTagPosition != null && aprilTagPosition.tag_Z > zMin) {
                validTags++;
            }
        }
        //if two or more tags are "valid" then it states the hive is facing up.
        return validTags >= 1;
    }

    public boolean hiveFacingUp(boolean redHive, boolean frontHive){
        if(redHive && frontHive){ return hiveFacingUp(frontRed);}
        else if(redHive && !frontHive){ return hiveFacingUp(backRed);}
        else if(!redHive && frontHive){ return hiveFacingUp(frontBlue);}
        else { return hiveFacingUp(backBlue);}
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        getAprilTags();


    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("backBlue up: ",hiveFacingUp(backBlue));
        telemetry.addData("frontBlue up: ",hiveFacingUp(frontBlue));
        telemetry.addData("backRed up: ",hiveFacingUp(backRed));
        telemetry.addData("frontRed up: ",hiveFacingUp(frontRed));
    }

}
