package org.firstinspires.ftc.teamcode.opmodes.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.AutoOpModeBase;

@Autonomous(name = "_Red > Start F2", preselectTeleOp="MainTeleop")
public class Red_F3 extends AutoOpModeBase {

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void executeOpMode() {

        rollingIntake.SetElbowInIntakePosition();
        DeliverSample();

        driveTrain.DriveToTarget(400, 200);
        Wait(300);
        driveTrain.TurnAngle(0);
        Wait(300);

        if (!driveTrain.AlignTx()) {
            telemetry.addLine("Cannot find sample");
            telemetry.update();
        }

        driveTrain.AlignTy();

        driveTrain.TurnRelative(20);
        driveTrain.Forward(60);

        pivot.MoveToIntakeInAuto();

        rollingIntake.IntakeInAuto();
        Wait(300);

        pivot.MoveToIntakeSampleInAuto();
        Wait(500);

        rollingIntake.HoldInAuto();

        pivot.MoveToDeliveryInAuto();

        DeliverSample();

        driveTrain.DriveToTarget(1200, 200);
    }

    private void DeliverSample() {
        driveTrain.DriveToTarget(350, 400);
        driveTrain.TurnAngle(125);

        pivot.MoveToDeliveryInAuto();
        slider.ExtendMaxInAuto();

        driveTrain.DriveToTarget(250, 400);
        driveTrain.TurnAngle(140);

        rollingIntake.OuttakeInAuto();
        Wait(1500);
        rollingIntake.HoldInAuto();

        driveTrain.DriveToTarget(350, 400);
        driveTrain.TurnAngle(125);

        slider.CollapseMinInAuto();
        pivot.MoveToStartInAuto();
    }

}