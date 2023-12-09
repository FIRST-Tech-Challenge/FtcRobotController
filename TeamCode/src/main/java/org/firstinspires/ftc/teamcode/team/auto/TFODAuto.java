package org.firstinspires.ftc.teamcode.team.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.team.CSVP;


@Autonomous(name = "TFODAuto", group = "Test")
public class TFODAuto extends LinearOpMode {
    CSBaseLIO drive;
    private static double dt;
    private static TimeProfiler updateRuntime;
    private Recognition finalRecog = null;

    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    CSVP CSVP;

    enum State {
        DETECT,
        PARK
    }

    State currentState = State.DETECT;

    public void runOpMode() throws InterruptedException{
        setUpdateRuntime(new TimeProfiler(false));

        drive = new CSBaseLIO(hardwareMap);

        drive.getExpansionHubs().update(getDt());

        double t1 = waitTimer.milliseconds();

        CSVP = new CSVP();
        CSVP.initTfod(hardwareMap);

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        int detectCounter = 0;
        double confidence = 0;
        String label = "NONE";
        Recognition oldRecog = null;
        Recognition recog;

        waitForStart();
        double start = waitTimer.milliseconds();
        while (opModeIsActive() && !isStopRequested()){
            switch (currentState){
                case DETECT:
                    recog = CSVP.detect();
                    detectCounter++;
                    if (recog != null){
                        if(oldRecog != null) {
                            if (CSVP.detect() == recog){
                                confidence = recog.getConfidence();
                                label = recog.getLabel();
                                oldRecog = recog;
                            }
                        }
                        else{
                            oldRecog = recog;
                        }
                    }
                    else {
                        telemetry.addLine("NULL");
//                        telemetry.addData("Label: ", recog.getLabel());
                    }
                    if (waitTimer.milliseconds() > start + 5000 && confidence > 0.01){
                        currentState = State.PARK;
                    }
                    break;

                case PARK:
                    telemetry.addData("Label: ", label);
                    telemetry.addData("Confidence: ", confidence);
                    telemetry.addData("#: ", detectCounter);
                    break;
            }
            telemetry.update();
        }
        if (isStopRequested()) return;
    }

    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updaRuntime) {
        updateRuntime = updaRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double pdt) {
        dt = pdt;
    }
}
