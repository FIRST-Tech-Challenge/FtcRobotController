import { Middleware } from 'redux';

import { RootState, AppThunkDispatch } from '@/store/reducers';
import {
  receiveConnectionStatus,
  receivePingTime,
} from '@/store/actions/socket';
import {
  GET_ROBOT_STATUS,
  INIT_OP_MODE,
  RECEIVE_GAMEPAD_STATE,
  RECEIVE_ROBOT_STATUS,
  START_OP_MODE,
  STOP_OP_MODE,
} from '@/store/types';

let socket: WebSocket;
let statusSentTime: number;

export function startSocketWatcher(dispatch: AppThunkDispatch) {
  setInterval(() => {
    if (socket === undefined || socket.readyState === WebSocket.CLOSED) {
      socket = new WebSocket(
        `ws://${
          import.meta.env['VITE_REACT_APP_HOST'] || window.location.hostname
        }:${import.meta.env['VITE_REACT_APP_PORT']}`,
      );

      socket.onmessage = (evt) => {
        const msg = JSON.parse(evt.data);
        dispatch(msg);
      };

      socket.onopen = () => {
        dispatch(receiveConnectionStatus(true));
      };

      socket.onclose = () => {
        dispatch(receiveConnectionStatus(false));
      };
    } else if (socket.readyState === WebSocket.OPEN) {
      statusSentTime = Date.now();
      socket.send(JSON.stringify({ type: 'GET_ROBOT_STATUS' }));
    }

    dispatch(receiveConnectionStatus(socket.readyState === WebSocket.OPEN));
  }, 1000);
}

const socketMiddleware: Middleware<Record<string, unknown>, RootState> =
  (store) => (next) => (action) => {
    switch (action.type) {
      case RECEIVE_ROBOT_STATUS: {
        const pingTime = Date.now() - statusSentTime;
        store.dispatch(receivePingTime(pingTime));

        next(action);

        break;
      }
      // messages forwarded to the server
      case RECEIVE_GAMEPAD_STATE:
      case GET_ROBOT_STATUS:
      case 'SAVE_CONFIG':
      case 'GET_CONFIG':
      case INIT_OP_MODE:
      case START_OP_MODE:
      case STOP_OP_MODE: {
        if (socket !== undefined && socket.readyState === WebSocket.OPEN) {
          socket.send(JSON.stringify(action));
        }

        next(action);

        break;
      }
      default:
        next(action);

        break;
    }
  };

export default socketMiddleware;
