import {
  StatusState,
  ReceiveOpModeListAction,
  ReceiveRobotStatusAction,
  RECEIVE_OP_MODE_LIST,
  RECEIVE_ROBOT_STATUS,
} from '@/store/types';

export const receiveRobotStatus = (
  status: StatusState,
): ReceiveRobotStatusAction => ({
  type: RECEIVE_ROBOT_STATUS,
  status,
});

export const receiveOpModeList = (
  opModeList: string[],
): ReceiveOpModeListAction => ({
  type: RECEIVE_OP_MODE_LIST,
  opModeList,
});
