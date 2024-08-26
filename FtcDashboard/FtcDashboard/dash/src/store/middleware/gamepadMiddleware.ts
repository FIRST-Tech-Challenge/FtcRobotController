import { Middleware } from 'redux';

/*
 * Some of this gamepad logic is based on FTC Team 731's robotics simulator.
 * https://github.com/nicholasday/robotics-simulator
 */

import {
  gamepadConnected,
  gamepadDisconnected,
  sendGamepadState,
} from '@/store/actions/gamepad';
import { GamepadState, GAMEPAD_SUPPORTED_STATUS } from '@/store/types';
import { AppThunkDispatch, RootState } from '@/store/reducers';
import GamepadType from '@/enums/GamepadType';

const scale = (
  value: number,
  oldMin: number,
  oldMax: number,
  newMin: number,
  newMax: number,
) => newMin + ((value - oldMin) * (newMax - newMin)) / (oldMax - oldMin);

// based on the corresponding function in the SDK Gamepad
const cleanMotionValues = (
  value: number,
  joystickDeadzone = 0.2,
  maxMotionRange = 1.0,
) => {
  // apply deadzone
  if (-joystickDeadzone < value && value < joystickDeadzone) return 0;

  // apply trim
  if (value > maxMotionRange) return maxMotionRange;
  if (value < -maxMotionRange) return maxMotionRange;

  // scale values between deadzone and trim to 0 and max range
  if (value > 0) {
    return scale(value, joystickDeadzone, maxMotionRange, 0, maxMotionRange);
  } else {
    return scale(value, -joystickDeadzone, -maxMotionRange, 0, -maxMotionRange);
  }
};

const REST_GAMEPAD_STATE: GamepadState = {
  left_stick_x: 0,
  left_stick_y: 0,
  right_stick_x: 0,
  right_stick_y: 0,

  dpad_up: false,
  dpad_down: false,
  dpad_left: false,
  dpad_right: false,

  a: false,
  b: false,
  x: false,
  y: false,

  guide: false,
  start: false,
  back: false,

  left_bumper: false,
  right_bumper: false,

  left_stick_button: false,
  right_stick_button: false,

  left_trigger: 0,
  right_trigger: 0,
};

const extractGamepadState = (gamepad: Gamepad) => {
  const type = GamepadType.getFromGamepad(gamepad);
  if (!GamepadType.isSupported(type)) {
    throw new Error('Unable to extract state from unsupported gamepad.');
  }

  switch (type) {
    case GamepadType.LOGITECH_DUAL_ACTION:
      return {
        left_stick_x: cleanMotionValues(-gamepad.axes[1]),
        left_stick_y: cleanMotionValues(gamepad.axes[2]),
        right_stick_x: cleanMotionValues(-gamepad.axes[3]),
        right_stick_y: cleanMotionValues(gamepad.axes[4]),

        dpad_up: gamepad.buttons[12].pressed,
        dpad_down: gamepad.buttons[13].pressed,
        dpad_left: gamepad.buttons[14].pressed,
        dpad_right: gamepad.buttons[15].pressed,

        a: gamepad.buttons[1].pressed,
        b: gamepad.buttons[2].pressed,
        x: gamepad.buttons[0].pressed,
        y: gamepad.buttons[3].pressed,

        guide: false,
        start: gamepad.buttons[9].pressed,
        back: gamepad.buttons[8].pressed,

        left_bumper: gamepad.buttons[4].pressed,
        right_bumper: gamepad.buttons[5].pressed,

        left_stick_button: gamepad.buttons[10].pressed,
        right_stick_button: gamepad.buttons[11].pressed,

        left_trigger: gamepad.buttons[6].value,
        right_trigger: gamepad.buttons[7].value,
      };
    case GamepadType.XBOX_360:
      return {
        // same as SONY_DUALSHOCK_4 except guide and touchpad buttons
        // tested with generic controller id='Xbox 360 Controller (XInput STANDARD GAMEPAD)' and mapping='standard'
        // on Win10/Chrome v88 and Edge v87
        // USB ID=24C6, PID=530A
        left_stick_x: cleanMotionValues(gamepad.axes[0]),
        left_stick_y: cleanMotionValues(gamepad.axes[1]),
        right_stick_x: cleanMotionValues(gamepad.axes[3]),
        right_stick_y: cleanMotionValues(gamepad.axes[4]),

        dpad_up: gamepad.buttons[12].pressed,
        dpad_down: gamepad.buttons[13].pressed,
        dpad_left: gamepad.buttons[14].pressed,
        dpad_right: gamepad.buttons[15].pressed,

        a: gamepad.buttons[0].pressed,
        b: gamepad.buttons[1].pressed,
        x: gamepad.buttons[2].pressed,
        y: gamepad.buttons[3].pressed,

        guide: false,
        start: gamepad.buttons[9].pressed,
        back: gamepad.buttons[8].pressed,

        left_bumper: gamepad.buttons[4].pressed,
        right_bumper: gamepad.buttons[5].pressed,

        left_stick_button: gamepad.buttons[10].pressed,
        right_stick_button: gamepad.buttons[11].pressed,
        left_trigger: gamepad.buttons[6].value,
        right_trigger: gamepad.buttons[7].value,
      };
    case GamepadType.SONY_DUALSHOCK_4:
      {
        const state: GamepadState = {
          left_stick_x: cleanMotionValues(gamepad.axes[0]),
          left_stick_y: cleanMotionValues(gamepad.axes[1]),
          right_stick_x: cleanMotionValues(gamepad.axes[2]),
          right_stick_y: cleanMotionValues(gamepad.axes[3]),

          dpad_up: gamepad.buttons[12].pressed,
          dpad_down: gamepad.buttons[13].pressed,
          dpad_left: gamepad.buttons[14].pressed,
          dpad_right: gamepad.buttons[15].pressed,

          a: gamepad.buttons[0].pressed,
          b: gamepad.buttons[1].pressed,
          x: gamepad.buttons[2].pressed,
          y: gamepad.buttons[3].pressed,

          guide: gamepad.buttons[16].pressed,
          start: gamepad.buttons[9].pressed,
          back: gamepad.buttons[8].pressed,

          left_bumper: gamepad.buttons[4].pressed,
          right_bumper: gamepad.buttons[5].pressed,

          left_stick_button: gamepad.buttons[10].pressed,
          right_stick_button: gamepad.buttons[11].pressed,
          left_trigger: gamepad.buttons[6].value,
          right_trigger: gamepad.buttons[7].value,
        };
        if (gamepad.buttons[17] !== undefined) {
          state.touchpad = gamepad.buttons[17].pressed;
        }
        return state;
      }
      break;

    default:
      throw new Error(`Unable to handle support gamepad of type ${type}`);
  }
};

let gamepad1Index = -1;
let gamepad2Index = -1;

const gamepadMiddleware: Middleware<Record<string, unknown>, RootState> = (
  store,
) => {
  let getGamepads = navigator.getGamepads?.bind(navigator);
  if (getGamepads == null) {
    getGamepads = function () {
      return [null, null, null, null];
    };
    console.log(
      'Gamepads not supported over non-https. See https://developer.mozilla.org/en-US/docs/Web/API/Gamepad',
    );
    setTimeout(() => {
      store.dispatch({
        type: GAMEPAD_SUPPORTED_STATUS,
        gamepadsSupported: false,
      });
    }, 1000);
  } else {
    setTimeout(() => {
      store.dispatch({
        type: GAMEPAD_SUPPORTED_STATUS,
        gamepadsSupported: true,
      });
    }, 1000);
  }
  function updateGamepads() {
    const gamepads = getGamepads();
    if (gamepads.length === 0) {
      setTimeout(updateGamepads, 500);
      return;
    }

    // check for Start-A/Start-B
    for (const gamepad of getGamepads()) {
      if (gamepad === null || !gamepad.connected) {
        continue;
      }

      const gamepadType = GamepadType.getFromGamepad(gamepad);
      if (!GamepadType.isSupported(gamepadType)) {
        continue;
      }

      const gamepadState = extractGamepadState(gamepad);

      // update gamepad 1 & 2 associations
      if (gamepadState.start && gamepadState.a) {
        gamepad1Index = gamepad.index;

        store.dispatch(gamepadConnected(1));

        if (gamepad2Index === gamepad1Index) {
          store.dispatch(gamepadDisconnected(2));

          gamepad2Index = -1;
        }
      } else if (gamepadState.start && gamepadState.b) {
        gamepad2Index = gamepad.index;

        store.dispatch(gamepadConnected(2));

        if (gamepad1Index === gamepad2Index) {
          store.dispatch(gamepadDisconnected(1));

          gamepad1Index = -1;
        }
      }

      // actually dispatch motion events
      let gamepad1State;
      if (gamepad1Index !== -1) {
        const gamepad = gamepads[gamepad1Index];

        if (gamepad) {
          gamepad1State = extractGamepadState(gamepad);
        } else {
          gamepad1State = REST_GAMEPAD_STATE;
        }
      } else {
        gamepad1State = REST_GAMEPAD_STATE;
      }

      let gamepad2State;
      if (gamepad2Index !== -1) {
        const gamepad = gamepads[gamepad2Index];

        if (gamepad) {
          gamepad2State = extractGamepadState(gamepad);
        } else {
          gamepad2State = REST_GAMEPAD_STATE;
        }
      } else {
        gamepad2State = REST_GAMEPAD_STATE;
      }

      (store.dispatch as AppThunkDispatch)(
        sendGamepadState(gamepad1State, gamepad2State),
      );
    }

    requestAnimationFrame(updateGamepads);
  }

  window.addEventListener('gamepaddisconnected', (evt: Event) => {
    // Required because lib.dom.d.ts doesn't have proper types for the gamepad events
    // Looks like it's fixed but currently not merged in the version we are using
    // See: https://github.com/microsoft/TypeScript/issues/39425 & https://github.com/microsoft/TypeScript-DOM-lib-generator/pull/925
    const { gamepad } = evt as GamepadEvent;

    if (gamepad1Index === gamepad.index) {
      store.dispatch(gamepadDisconnected(gamepad1Index));

      gamepad1Index = -1;
    } else if (gamepad2Index === gamepad.index) {
      store.dispatch(gamepadDisconnected(gamepad2Index));

      gamepad2Index = -1;
    }
  });

  updateGamepads();

  return (next) => (action) => next(action);
};

export default gamepadMiddleware;
