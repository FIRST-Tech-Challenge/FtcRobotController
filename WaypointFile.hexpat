// Gear Up waypoints file
#pragma endian big

struct Waypoint2 {
    double x [[color("ff8080")]];
    double y [[color("8080ff")]];
};
struct Waypoint3 {
    double x [[color("ff8080")]];
    double y [[color("80ff80")]];
    double r [[color("8080ff")]];
};

fn explode() {
    u8 x = 0/0;
};

struct File {
    char magic[2];
    s8 size;
    s32 count;
    if (size == 2)
        Waypoint2 waypoints[count];
    else if (size == 3)
        Waypoint3 waypoints[count];
    else explode();
};

File main @ 0x00;