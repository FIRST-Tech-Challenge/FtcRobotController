import OpModeStatus from '@/enums/OpModeStatus';
import { Values } from '@/typeHelpers';

export const GET_ROBOT_STATUS = 'GET_ROBOT_STATUS';
export const RECEIVE_ROBOT_STATUS = 'RECEIVE_ROBOT_STATUS';
export const RECEIVE_OP_MODE_LIST = 'RECEIVE_OP_MODE_LIST';
export const GAMEPAD_SUPPORTED_STATUS = 'GAMEPAD_SUPPORTED_STATUS';

export type RobotStatus = {
  enabled: boolean;
  available: boolean;
  activeOpMode: string;
  activeOpModeStatus: Values<typeof OpModeStatus>;
  warningMessage: string;
  errorMessage: string;
  batteryVoltage: number;
};

export type StatusState = {
  enabled: boolean;
  available: boolean;
  activeOpMode: string;
  activeOpModeStatus: Values<typeof OpModeStatus>;
  warningMessage: string;
  errorMessage: string;
  batteryVoltage: number;
  opModeList: string[];
  gamepadsSupported: boolean;
};

export type GetRobotStatusAction = {
  type: typeof GET_ROBOT_STATUS;
};

export type ReceiveRobotStatusAction = {
  type: typeof RECEIVE_ROBOT_STATUS;
  status: RobotStatus;
};

export type ReceiveOpModeListAction = {
  type: typeof RECEIVE_OP_MODE_LIST;
  opModeList: string[];
};

export type GamepadSupportedStatus = {
  type: typeof GAMEPAD_SUPPORTED_STATUS;
  gamepadsSupported: boolean;
};
