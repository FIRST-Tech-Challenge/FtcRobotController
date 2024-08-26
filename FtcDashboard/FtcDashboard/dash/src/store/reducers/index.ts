import { ThunkAction, ThunkDispatch } from 'redux-thunk';
import { Action, combineReducers } from 'redux';

import telemetryReducer from './telemetry';
import socketReducer from './socket';
import configReducer from './config';
import statusReducer from './status';
import cameraReducer from './camera';
import settingsReducer from './settings';
import gamepadReducer from './gamepad';
import { createDispatchHook } from 'react-redux';

const rootReducer = combineReducers({
  telemetry: telemetryReducer,
  socket: socketReducer,
  config: configReducer,
  status: statusReducer,
  camera: cameraReducer,
  settings: settingsReducer,
  gamepad: gamepadReducer,
});

export type RootState = ReturnType<typeof rootReducer>;
export type RootActions = Parameters<typeof rootReducer>[1];

export const useAppDispatch = createDispatchHook<RootState, RootActions>();

// TODO: these types seem to only be used in the middlewares
// but there we have the freedom to dispatch how we like?
// not sure why we need thunks there tbh
export type AppThunkAction<ReturnType = void> = ThunkAction<
  ReturnType,
  RootState,
  unknown,
  Action<string>
>;

export type AppThunkDispatch<ReturnType = void> = ThunkDispatch<
  RootState,
  ReturnType,
  Action
>;

export default rootReducer;
