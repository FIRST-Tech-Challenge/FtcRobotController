// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.CompBot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Displays a graphic to the driver station */
public class GraphicTelemetry {

  public LinearOpMode myOp;

  GraphicTelemetry(LinearOpMode opMode) {
    myOp = opMode;
  }

  TelemetryPacket packet = new TelemetryPacket();
  Canvas fieldOverlay = packet.fieldOverlay();

  FtcDashboard dashBoard = FtcDashboard.getInstance();

  /**
   * Displays the four wheels at the angle that they are set to
   *
   * @param FLWheelAngle Angle of the Front Left Wheel
   * @param BLWheelAngle Angle of the Back Left Wheel
   * @param FRWheelAngle Angle of the Front Right Wheel
   * @param BRWheelAngle Angle of the Back Right wheel
   */
  public void wheelAnglesGraphic(
      double FLWheelAngle, double BLWheelAngle, double FRWheelAngle, double BRWheelAngle) {

    int wheelWidth = 20;
    int wheelHeight = 40;

    fieldOverlay.clear();
    fieldOverlay.setStroke("blue");

    dashBoard.sendTelemetryPacket(packet); // Sends the current frame to the display
  }
}
