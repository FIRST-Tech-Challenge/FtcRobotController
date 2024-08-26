import { applyMiddleware, createStore } from 'redux';
import { createLogger } from 'redux-logger';
import thunk from 'redux-thunk';

import gamepadMiddleware from './middleware/gamepadMiddleware';
import socketMiddleware from './middleware/socketMiddleware';
import storageMiddleware from './middleware/storageMiddleware';
import rootReducer from './reducers';
import {
  GET_ROBOT_STATUS,
  RECEIVE_PING_TIME,
  RECEIVE_ROBOT_STATUS,
  RECEIVE_TELEMETRY,
} from './types';

const HIDDEN_ACTIONS = [
  RECEIVE_PING_TIME,
  RECEIVE_TELEMETRY,
  RECEIVE_ROBOT_STATUS,
  GET_ROBOT_STATUS,
];

const configureStore = () => {
  const middlewares = [
    thunk,
    gamepadMiddleware,
    socketMiddleware,
    storageMiddleware,
  ];

  if (import.meta.env.DEV) {
    const logger = createLogger({
      predicate: (getState, action) =>
        HIDDEN_ACTIONS.indexOf(action.type) === -1,
    });

    middlewares.push(logger);
  }

  return createStore(rootReducer, applyMiddleware(...middlewares));
};

export default configureStore;
