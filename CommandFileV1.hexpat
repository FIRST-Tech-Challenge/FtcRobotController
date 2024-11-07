#pragma endian big

enum Kind: u8 {
    Move = 0x00,
    Run = 0x01,
    RunAsync = 0x02,
    Await = 0x03
};

struct MoveMixin {
    double x [[color("800000")]];
    double y [[color("008000")]];
    double r [[color("000080")]];
} [[inline]];
struct RPCMixin {
    s32 size [[color("800000")]];
    char content[size] [[color("80bbbbb")]];
} [[inline]];

struct Command {
    Kind kind [[color("b080ff")]];
    if (kind == Kind::Move) MoveMixin content;
    if (kind == Kind::Run) RPCMixin content;
    if (kind == Kind::RunAsync) RPCMixin content;
    if (kind == Kind::Await) RPCMixin content;
};

struct File {
    char magic[2];
    s32 count;
    Command commands[count];
};

File main @ 0x00;