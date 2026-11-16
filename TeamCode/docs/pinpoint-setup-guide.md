# Pinpoint Odometry: Setup and Test Guide

For the Reed City Cyber Coyotes (teams 11940 and 22091, and the practice bot). The code is `PinpointSubsystem`. The test OpMode is **Pinpoint - Alpha**. The same steps work on every robot, because we use the same chassis. Do them once on each robot.

**What it is:** the goBILDA Pinpoint is a small computer that reads our two spring-loaded (swingarm) odometry pods and its own gyro, and reports where the robot is: **X** (inches forward), **Y** (inches left), and **heading** (degrees, turning left is positive).

## Step 0: What last season changed

Last season's code named the device **`odo`**. This season's code names it **`pinpoint`** (the name the SDK sample uses). So each robot's configuration needs a one-time edit. Nothing else on the robot changes: the pods and the Pinpoint are already wired.

## Step 1: Driver Station configuration (once per robot)

Do this with the Driver Station connected to the Control Hub.

1. Open the three-dot menu on the Driver Station, then **Configure Robot**.
2. Open your robot's configuration and tap **Edit**. (Make a copy first if you want a backup of the old one.)
3. Tap **Control Hub Portal**, then **I2C**.
4. Find the Pinpoint. It is on **bus 1, 2 or 3** (bus 0 is the Control Hub's own built-in gyro, so it will not be the Pinpoint).
   - If last season's `odo` is already there, tap its name and change it to **`pinpoint`**. Leave the bus alone.
   - If nothing is on that bus, tap the bus number, then **Add**, and choose **goBILDA Pinpoint Odometry Computer** (the name in the list may have a ® mark). Name it **`pinpoint`**.
   - To find which bus the cable is on, look at which I2C port on the Hub the Pinpoint's cable goes into. Write it on the robot's wiring sheet (`biobuzz-robot-config-quick-reference.md`).
5. The name must be **exactly** `pinpoint`, all lowercase.
6. Tap **Done**, **Save**, and make that configuration **Active** (the one the Driver Station shows).

If you skip this, nothing breaks. `TeleOp - Alpha 15` and `Pinpoint - Alpha` still run, and the screen just says **Pinpoint: NOT FOUND**.

## Step 2: Deploy and run the test

1. Deploy the code from Android Studio.
2. Put the robot on the floor, **still**. Select **Pinpoint - Alpha** and press INIT.
3. Work through the numbered test order in the OpMode's header (steps 1 to 6). In short:

| Test | What you do | What you should see |
|---|---|---|
| 1. Status | Look at the screen after INIT | `READY` |
| 2. Pod directions | Press **A**, push the robot **forward**, then push it **left** | **X** goes up when pushed forward, and **Y** goes up when pushed left. (The pod tick lines are raw counts before the direction setting, so judge by X and Y. They only need to change when you push.) |
| 3. Heading sign | Press **B** (robot still), then turn the robot **left** by hand | **Heading** goes **up** |
| 4. Distance | Press **A**, push the robot 48 inches along a tape measure | **X** shows about 48 |
| 5. Turn in place | Press **A**, spin the robot 360 degrees left around its middle | Heading, X and Y all end near 0 |
| 6. Drift | Push it around a minute, then bring it back to the start | X, Y and heading end near 0 (a few inches is normal) |

## Step 3: Fix what the tests find

Every fix is one constant at the top of `PinpointSubsystem.java`. Change it, redeploy, and run the test again.

| Problem | Constant to change |
|---|---|
| **X** or **Y** goes **down** when it should go up | `FORWARD_POD_DIRECTION` or `SIDEWAYS_POD_DIRECTION`: change `FORWARD` to `REVERSED` for that pod |
| Heading goes **down** when you turn left | `INVERT_HEADING`: change `false` to `true` |
| X or Y is off by a steady amount (for example it shows 32 for a real 48) | `DISTANCE_CALIBRATION_MULTIPLIER`: real distance divided by shown distance (48 / 32 = 1.5) |
| X and Y wander when you only spin the robot | `FORWARD_POD_SIDEWAYS_OFFSET_INCHES` and `SIDEWAYS_POD_FORWARD_OFFSET_INCHES`: remeasure with a tape measure (see below) |
| Status says `FAULT: ... pod not detected` | Check that pod's cable at the Pinpoint |
| Status says `NOT FOUND` | The name or the configuration is wrong (Step 1) |

**Measuring the pod offsets.** Find the robot's **center of turning** (the middle of the drivetrain, where it spins if you turn in place). Measure from there to the middle of each pod's wheel.
- **Forward pod** (counts forward and backward): how far **sideways** from center. Left is positive, right is negative. Ours: 3.75 inches right, so **-3.75**.
- **Sideways pod** (counts left and right): how far **forward** from center. Forward is positive, behind is negative. Ours: centered, so **0.0**.

These numbers came from last season's code on the same chassis. Remeasure if either pod has been moved.

## Things to know

- **Keep the robot still during INIT.** The Pinpoint calibrates its gyro then. If you bump it, press **B** in `Pinpoint - Alpha`, or INIT again.
- **The pods must press on the floor.** If a pod lifts (bumping over a game piece, or a loose spring), the readings are wrong.
- **It drifts a little.** Over a whole match the position can be off by a few inches. Treat it as close, not exact.
- **Last season's distance fix was not copied.** Last season's helper multiplied distances by 1.5. We start at 1.0 and measure, because 1.5 is exactly the ratio between goBILDA's two pod settings, so it probably covered up a setting that was not taking effect.
- **Last season's heading flip was not copied either.** Test 3 decides it.
- **In Autonomous,** Pedro Pathing reads the Pinpoint by itself. Never run `PinpointSubsystem` and a Pedro Follower in the same OpMode.
