import {
  SocketState,
  ReceiveConnectionStatusAction,
  ReceivePingTimeAction,
  RECEIVE_CONNECTION_STATUS,
  RECEIVE_PING_TIME,
} from '@/store/types';

const initialState: SocketState = {
  isConnected: false,
  pingTime: 0,
};

const socketReducer = (
  state = initialState,
  action: ReceivePingTimeAction | ReceiveConnectionStatusAction,
) => {
  switch (action.type) {
    case RECEIVE_PING_TIME:
      return {
        ...state,
        pingTime: action.pingTime,
      };
    case RECEIVE_CONNECTION_STATUS:
      return {
        ...state,
        isConnected: action.isConnected,
      };
    default:
      return state;
  }
};

export default socketReducer;
