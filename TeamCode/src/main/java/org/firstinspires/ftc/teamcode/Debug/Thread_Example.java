// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Messing around with multithreading. Don't touch this. I mean it. Don't */
public class Thread_Example extends LinearOpMode {

  Thread thread1 =
      new Thread() {
        public void run() {
          // sop (system out print)
        }
      };

  Thread thread2 =
      new Thread() {
        public void run() {
          // sop (system out print)
        }
      };

  @Override
  public void runOpMode() throws InterruptedException {
    thread1.start();

    thread2.start();
  }
}
