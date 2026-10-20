# 0006: Write the code for students to read

Status: Accepted.
Date: 2026-09-27

## Context

The team is 6th to 8th graders. They learn by reading the team's code, then changing it.
Code that is correct but hard to follow doesn't help them: short names, nested calls and
extra layers all make them give up before they reach the robot part.

## Decision

Code in `TeamCode/src/main` is written to be read by a student first.

- **Names say what the thing is.** `mFrontLeftMotor`, not `mMotDrFrLt`.
  `mResetHeadingButton`, not `mCircle`. `mSpeedLimit`, not `mMaxPower`.
- **One step per line.** Put a middle result in a named variable instead of nesting calls:
  `double safePower = LimitPower(power, mOp.mSafeMode);` then use `safePower`.
- **Return the answer.** `powers = MecanumMath.FieldCentric(...)`, not a method that fills
  in an object you pass it.
- **Few layers.** Logic lives where it's used. A one-method helper class only exists if
  several subsystems share it. Two short copies of a formula beat a helper with six
  parameters.
- **Plain Java.** No lambdas, streams, `? :` chains or bit tricks in robot code. Tests
  can use them.
- **Keep what students already know.** Team style stays: `m` before field names,
  method names start with a capital (`Init`, `Process`). Public calls last season's
  students used (`Debouncer.In/Out/Used`) keep their names.
- **Speed is not a reason** to make the code harder to read. The loop runs about 50 times
  a second; a few small objects per loop don't matter.

## Consequences

- Some code repeats itself on purpose. `MecanumMath.RobotCentric` and `FieldCentric` each
  spell out all four wheel formulas.
- Tests still check the behavior. The readability pass that introduced this decision
  changed no behavior: the wheel math still matches Decode's on 400,000 random inputs.
- Review question for every change: "Could a student who knows `if` and `for` follow this?"
