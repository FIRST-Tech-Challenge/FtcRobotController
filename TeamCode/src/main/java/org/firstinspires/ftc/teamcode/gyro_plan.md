encoderDrive

angle, magnitude

rolling average of angle deviation?

algorithm

angle T, magnitude X
calculate theoretical DT
move
measure actual DT and t
calculate % chg T
find DT/t
repeat for all movements
at same time calculate weighted average (linear increase in t = linear increase in weight)
for each new movement adjust robot angle by opposite of weighted average of errors
