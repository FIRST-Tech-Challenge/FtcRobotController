package org.firstinspires.ftc.teamcode.Common;

public class MatchState {

    private static MatchState instance;

    private Alliance alliance;

    private MatchState() {
        alliance = Alliance.Blue;
    }

    public static MatchState getInstance() {
        if (instance == null) {
            instance = new MatchState();
        }
        return instance;
    }

    public void initializeForMatchWithGamepad(MyRobot robot) {
        if (robot.gamepad1().dpadLeftWasReleased()) {
            alliance = Alliance.Blue;
        }
        if (robot.gamepad1().dpadRightWasReleased()) {
            alliance = Alliance.Red;
        }

        robot.telemetry().addData("Alliance: ", alliance);
        robot.telemetry().update();
    }

    public Alliance getAlliance() {
        return alliance;
    }
}
