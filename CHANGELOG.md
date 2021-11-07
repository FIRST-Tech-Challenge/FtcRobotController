# Changelog

All notable changes to this project will be documented in this file. See [standard-version](https://github.com/conventional-changelog/standard-version) for commit guidelines.

### 0.0.1 (2021-11-07)


### ⚠ BREAKING CHANGES

* refactor so drive system in operation mode is directly the one specified and not generic

### Features

* add autonomous operation mode ([dde936e](https://github.com/Nick-Fanelli/FtcRobotController/commit/dde936e145913de379db43d0224097d5ee72dc1a))
* add basic operation mode and robot class ([04e48ac](https://github.com/Nick-Fanelli/FtcRobotController/commit/04e48ac2d15e7d252d25a3a14ba4c908f4e268be))
* add bravenator runtime exception ([86880dd](https://github.com/Nick-Fanelli/FtcRobotController/commit/86880dd75f04137a15eaae1e7b522606e45e49ff))
* add configuration opmode ([c5c28a1](https://github.com/Nick-Fanelli/FtcRobotController/commit/c5c28a109eceabab0be3407275fda27631305948))
* add ease of use generate motors method to four wheel drive class ([3290b92](https://github.com/Nick-Fanelli/FtcRobotController/commit/3290b9232409375701305a7b9e79e4573d9fb131))
* add gamepad support in teleop operation mode ([465b51b](https://github.com/Nick-Fanelli/FtcRobotController/commit/465b51b6a09e5ae0726f66cc487acdd6cc817561))
* add in calculated position methods to four wheel drive ([8f4f4d7](https://github.com/Nick-Fanelli/FtcRobotController/commit/8f4f4d77c1c09756279b32842faf161c22109229))
* add in saftey check system to insure the robot specifications are configured correctly ([75db2ee](https://github.com/Nick-Fanelli/FtcRobotController/commit/75db2ee3ff648b2530ec83769925c21ee4103e49))
* add starting position to robot config ([e9403e8](https://github.com/Nick-Fanelli/FtcRobotController/commit/e9403e81562c92c1acaf3c41f2dbab5c772b1d15))
* add tensorflow object detector class (not fully implemented yet) ([aae8734](https://github.com/Nick-Fanelli/FtcRobotController/commit/aae87349c955059c5ffa0773ca73495c94d6917a))
* add unknown saftey to force the duck position to end in either left, right or center ([2727438](https://github.com/Nick-Fanelli/FtcRobotController/commit/2727438c554c19e8732280d689a0915898c236d4))
* enable use velocity as default ([ec08f8c](https://github.com/Nick-Fanelli/FtcRobotController/commit/ec08f8ca59f4232776d026be6d80f769fde6ff5d))
* four wheel drive turning ([47d7152](https://github.com/Nick-Fanelli/FtcRobotController/commit/47d71523d88e4c15a562bdc3455cabba06137b00))
* mecanum drive ([e7a9338](https://github.com/Nick-Fanelli/FtcRobotController/commit/e7a93382baac7a0504185c8ab2cae3bb4d1d2505))
* require wheel specs ([80a4ef0](https://github.com/Nick-Fanelli/FtcRobotController/commit/80a4ef068d7fca0380b0caa6c4d03aa9b66e5c76))
* saftey check for motor count mismatch ([4d2f7ce](https://github.com/Nick-Fanelli/FtcRobotController/commit/4d2f7ce62d06f76c239f597b78dd5900044a4acf))
* selectable drive system ([8c28612](https://github.com/Nick-Fanelli/FtcRobotController/commit/8c286120f094cfe6c03fc1fbcb0695a76aa78036))
* tensor flow object detection recognition support ([070b6e2](https://github.com/Nick-Fanelli/FtcRobotController/commit/070b6e284e037b98224b3f23a53925f61819eb0f))
* two wheel drive ([4fd2712](https://github.com/Nick-Fanelli/FtcRobotController/commit/4fd27120065ea744fb7410db7900d99bdb0e9ef8))


### Bug Fixes

* change the auto update recognition time delay to 5ms ([dfbdeb5](https://github.com/Nick-Fanelli/FtcRobotController/commit/dfbdeb5296f9af6e6513e483076eced6c6718a6d))
* create new motors before assigning them to a drive system ([c7188a8](https://github.com/Nick-Fanelli/FtcRobotController/commit/c7188a8963ac0374c774cea6f55e8e6ca86b9b89))
* create robot each operation mode ([0ab8b78](https://github.com/Nick-Fanelli/FtcRobotController/commit/0ab8b78570068321fc2a6380ea8dab30c4944896))
* deprecate unsupported FtcGamePad constructors ([c2bf32d](https://github.com/Nick-Fanelli/FtcRobotController/commit/c2bf32d5961063e8dfcd95c1e968ee1e87dff087))
* four wheel drive's drive method and robot creation ([5a80d86](https://github.com/Nick-Fanelli/FtcRobotController/commit/5a80d862523d1436a73680eb163790289bee2e1d))
* only reset the robot in auto mode ([a7dc1b9](https://github.com/Nick-Fanelli/FtcRobotController/commit/a7dc1b9f99fd6dd3616d2a97e5b980f45752029b))
* refactor shipping hub to warehouse ([40b4e7d](https://github.com/Nick-Fanelli/FtcRobotController/commit/40b4e7d043a27aa6fe9aaff81820353ccf051cde))
* refactor shipping hub to warehouse ([52bc5dd](https://github.com/Nick-Fanelli/FtcRobotController/commit/52bc5dd06fc927113b22872ee8769a9e771ce071))
* rename shipping hub to warehouse ([7398ea4](https://github.com/Nick-Fanelli/FtcRobotController/commit/7398ea43161c595280a10f3f2b56d08112144427))
* turning calculation ([dfadfd0](https://github.com/Nick-Fanelli/FtcRobotController/commit/dfadfd00ddb6d786ed16f0a9ae8b783d29703401))
* update to use velocity ([2067725](https://github.com/Nick-Fanelli/FtcRobotController/commit/20677250600e54947d95858aec80a3e3777c3c73))
* use new updated wheel diameter math to calculate turning distance ([490d283](https://github.com/Nick-Fanelli/FtcRobotController/commit/490d28331e0e266978872d30fa9159fa855739a2))


* refactor so drive system in operation mode is directly the one specified and not generic ([ff04230](https://github.com/Nick-Fanelli/FtcRobotController/commit/ff0423012d9357ace4cbcb91f864bb257b910518))
