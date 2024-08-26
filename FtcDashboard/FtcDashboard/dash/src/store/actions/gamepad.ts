import { isEqual } from 'lodash';
import { Dispatch } from 'redux';
import {
  GamepadConnectedAction,
  GamepadDisonnectedAction,
  ReceiveGamepadStateAction,
  GAMEPAD_CONNECTED,
  GAMEPAD_DISCONNECTED,
  RECEIVE_GAMEPAD_STATE,
  GamepadState,
} from '@/store/types';

export const gamepadConnected = (user: number): GamepadConnectedAction => ({
  type: GAMEPAD_CONNECTED,
  user,
});

export const gamepadDisconnected = (
  user: number,
): GamepadDisonnectedAction => ({
  type: GAMEPAD_DISCONNECTED,
  user,
});

export const receiveGamepadState = (
  gamepad1: GamepadState,
  gamepad2: GamepadState,
): ReceiveGamepadStateAction => ({
  type: RECEIVE_GAMEPAD_STATE,
  gamepad1,
  gamepad2,
});

/*
To save bandwidth, new gamepad states are only sent if they differ from the previous.
However, the dash regardless still sends gamepad messages at the rate below to feed
the watchdog on the RC (to reset the gamepads in case the connection is cut abruptly).
*/
const MAX_GAMEPAD_MS = 150;

let lastGamepad1: GamepadState;
let lastGamepad2: GamepadState;
let lastGamepadTimestamp: number;

export const sendGamepadState =
  (gamepad1: GamepadState, gamepad2: GamepadState) =>
  (dispatch: Dispatch<ReceiveGamepadStateAction>) => {
    const timestamp = Date.now();
    if (
      !isEqual(lastGamepad1, gamepad1) ||
      !isEqual(lastGamepad2, gamepad2) ||
      timestamp - lastGamepadTimestamp < MAX_GAMEPAD_MS
    ) {
      dispatch(receiveGamepadState(gamepad1, gamepad2));

      lastGamepad1 = gamepad1;
      lastGamepad2 = gamepad2;
      lastGamepadTimestamp = timestamp;
    }
  };
