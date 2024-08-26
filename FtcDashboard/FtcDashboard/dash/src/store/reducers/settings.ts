import {
  SettingState,
  ReceiveLayoutPresetAction,
  RECEIVE_LAYOUT_PRESET,
} from '@/store/types';

const initialState: SettingState = {
  // TODO: this seems to be necessary to prevent
  // ReferenceError: can't access lexical declaration 'LayoutPreset' before initialization
  // perhaps due to the root reducer type shenanigans?
  layoutPreset: 'DEFAULT',
};

const settingsReducer = (
  state: SettingState = initialState,
  action: ReceiveLayoutPresetAction,
) => {
  switch (action.type) {
    case RECEIVE_LAYOUT_PRESET:
      return {
        ...state,
        layoutPreset: action.preset,
      };
    default:
      return state;
  }
};

export default settingsReducer;
