algorithm for vuf line up

1. (not in game) detect vumark and find ideal position
2. (on game) initiate with method or with button press
2.5. optionally select the distance using d-pad to back up from the line
3. detect vumark
4. find position relative to vumark
5. calculate dx, dy, dz, dT
6. execute one of the following:
- encoder drive followed by encoder turn
- encoder drive and turn at the same time
7. reloop 3-6 optionally for better approximation
