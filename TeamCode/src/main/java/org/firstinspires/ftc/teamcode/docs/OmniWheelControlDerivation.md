# Omni Wheel Control Derivation
This made sense for me when looking at the three control axes individually, figuring out which way each wheel should turn (forward or reverse) to achieve the positive direction for that axis, and then summing the result for each axis at the end.

So, for each wheel:
1) Which way does the wheel need to turn to produce positive motion in the  **x axis**? If forward, write down "+x", if reverse, "-x"

    - For the front left wheel, forward rotation produces positive motion in the x axis, so we write "+x"

2) Which way does the wheel need to turn to produce positive motion in the  **y axis**? If forward, write down "+y", if reverse, "-y"

    - For the front left wheel, forward rotation produces positive motion in the x axis, so we write "+y"

3) Which way does the wheel need to turn to produce positive (clockwise) motion in the  **rotational (r) axis**? If forward, write down "+r", if reverse, "-r"

   - For the front left wheel, forward rotation produces positive motion in the r axis, so we write "+r"

4) Simply sum the combination of the x, y, and r axes! (and normalize the value between 0 and 1 by dividing by the largest value)

In my sketch below, you can see that if we give a pure x command (x = 1, y = 0, r = 0), the left wheels will turn away from each other and the right wheels will turn towards each other, which is exactly what we want!
<img width="953" alt="image" src="https://github.com/user-attachments/assets/8774de1f-a2a6-48d4-9964-218632dc4236" />
