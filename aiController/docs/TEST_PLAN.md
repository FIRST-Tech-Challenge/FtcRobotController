# Vision test plan / Plan de tests

Record date, camera, lighting, field setup, observed targets, and pass/fail for each test.

- [ ] Project builds and `Vision: Recognition Test` appears on the Driver Station.
- [ ] The configured `Webcam 1` provides a stream at the chosen resolution.
- [ ] With no game piece in view, color target says `none` most of the time.
- [ ] One matching game piece is detected; telemetry center moves smoothly as it moves.
- [ ] Largest matching object is selected when two pieces are visible.
- [ ] `horizontalError` is negative on the image's left and positive on its right.
- [ ] Non-target items, colored tape, and bright reflections are tested for false positives.
- [ ] Near/far, dark/bright lighting, and different backgrounds are tested.
- [ ] Known AprilTags appear with expected tag IDs; distant/occluded tags are tested.
- [ ] Robot remains stationary: this OpMode must not drive motors or servos.
- [ ] Camera closes and is available to a new OpMode after STOP.
- [ ] Optional HuskyLens path tested ONLY if hardware is present, wired, and trained.

## Evaluation log (copy per run)

| Setup | Real target? | Reported detection? | Correct label/ID? | Latency | Observation |
| --- | --- | --- | --- | --- | --- |
| Example: indoor light, 1 m | yes | yes | yes | not measured | center stable |

This is a checklist, **not** a record of tests that have already passed.
