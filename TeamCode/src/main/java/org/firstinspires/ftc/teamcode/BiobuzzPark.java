package org.firstinspires.ftc.teamcode;

import static org.lhssa.ftc.teamcode.pedroPathing.PedroTeleopData.allianceColor;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;
import org.lhssa.ftc.teamcode.pedroPathing.HeadingInterpolationType;
import org.lhssa.ftc.teamcode.pedroPathing.PedroMotion;
import org.lhssa.ftc.teamcode.pedroPathing.PedroPather;
import org.firstinspires.ftc.teamcode.teamPedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamPedroPathing.TeamPoses;

@Autonomous
public class BiobuzzPark extends OpMode {
    private PedroPather pedroPather;
    private PedroMotion pedroMotion;
    private RobotHardware robotHardware;
    private AllianceColor allianceColor = AllianceColor.RED;

    private Follower follower;
    private int currentAction = 1;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        pedroMotion = new PedroMotion(follower);
        follower.setPose(TeamPoses.startShootPose);

    }
    @Override
    public void init_loop(){
        menu();
    }
    @Override
    public void start(){
        pedroPather = new PedroPather(TeamPoses.canonicalColor, allianceColor, PedroPather::FlipRoutineAlliancesOnOppositeSides);

    }
    @Override
    public void loop() {
        follower.update();
        switch (currentAction){
            case 1:
                pedroMotion.goPath(pedroPather.pathBetween(TeamPoses.BlueStartTemp, TeamPoses.BlueParkTemp, HeadingInterpolationType.LINEAR));
                currentAction = 2;
                break;

        }

    }
    private void menu() {
        telemetry.addData("what alliance?", "UpDpad to toggle alliance");
        telemetry.addData("this is the currently selected alliance", allianceColor.toString());
        telemetry.addData("what side are you on", "DownDpad to toggle Side");

        telemetry.update();

        allianceColor = AllianceColor.INSTANCE.toggleColor(gamepad1.dpadUpWasPressed(), allianceColor);
    }
}
