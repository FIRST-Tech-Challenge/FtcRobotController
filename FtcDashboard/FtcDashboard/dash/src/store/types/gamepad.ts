export const GAMEPAD_CONNECTED = 'GAMEPAD_CONNECTED';
export const GAMEPAD_DISCONNECTED = 'GAMEPAD_DISCONNECTED';
export const RECEIVE_GAMEPAD_STATE = 'RECEIVE_GAMEPAD_STATE';

export type GamepadState = {
  left_stick_x: number;
  left_stick_y: number;
  right_stick_x: number;
  right_stick_y: number;

  dpad_up: boolean;
  dpad_down: boolean;
  dpad_left: boolean;
  dpad_right: boolean;

  a: boolean;
  b: boolean;
  x: boolean;
  y: boolean;

  guide: boolean;
  start: boolean;
  back: boolean;

  left_bumper: boolean;
  right_bumper: boolean;

  left_stick_button: boolean;
  right_stick_button: boolean;

  left_trigger: number;
  right_trigger: number;

  touchpad?: boolean;
};

export type GamepadConnectionState = {
  gamepad1Connected: boolean;
  gamepad2Connected: boolean;
};

export type GamepadConnectedAction = {
  type: typeof GAMEPAD_CONNECTED;
  user: number;
};

export type GamepadDisonnectedAction = {
  type: typeof GAMEPAD_DISCONNECTED;
  user: number;
};

export type ReceiveGamepadStateAction = {
  type: typeof RECEIVE_GAMEPAD_STATE;
  gamepad1: GamepadState;
  gamepad2: GamepadState;
};
