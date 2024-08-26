import {
  ConfigState,
  ConfigVar,
  ConfigVarState,
  ReceiveConfigAction,
  RefreshConfigAction,
  SaveConfigAction,
  UpdateConfigAction,
} from '@/store/types/config';

function inflate(v: ConfigVar): ConfigVarState {
  if (v.__type === 'custom') {
    const value = v.__value;
    if (value === null) {
      return {
        __type: 'custom',
        __value: null,
      };
    } else {
      return {
        __type: 'custom',
        __value: Object.keys(value).reduce(
          (acc, key) => ({
            ...acc,
            [key]: inflate(value[key]),
          }),
          {},
        ),
      };
    }
  } else {
    return {
      ...v,
      __newValue: v.__value,
      __valid: true,
    };
  }
}

// merge modified, matching members of base into latest
function mergeModified(
  base: ConfigVarState,
  latest: ConfigVar,
): ConfigVarState {
  if (base.__type === 'custom' && latest.__type === 'custom') {
    const latestValue = latest.__value;
    if (latestValue === null) {
      return {
        __type: 'custom',
        __value: null,
      };
    } else {
      return {
        __type: 'custom',
        __value: Object.keys(latestValue).reduce(
          (acc, key) =>
            base.__value !== null && key in base.__value
              ? {
                  ...acc,
                  [key]: mergeModified(base.__value[key], latestValue[key]),
                }
              : {
                  ...acc,
                  [key]: inflate(latestValue[key]),
                },
          {},
        ),
      };
    }
  } else if (
    base.__type === 'enum' &&
    latest.__type === 'enum' &&
    base.__enumClass === latest.__enumClass &&
    base.__value !== base.__newValue
  ) {
    return {
      ...base,
      __value: latest.__value,
    };
  } else if (
    base.__type === latest.__type &&
    /* type checker reminder */ base.__type !== 'custom' &&
    latest.__type !== 'custom' &&
    base.__value !== base.__newValue
  ) {
    return {
      ...base,
      __value: latest.__value,
    };
  } else {
    return inflate(latest);
  }
}

function revertModified(state: ConfigVarState): ConfigVarState {
  if (state.__type === 'custom') {
    const value = state.__value;
    if (value === null) {
      return {
        __type: 'custom',
        __value: null,
      };
    } else {
      return {
        __type: 'custom',
        __value: Object.keys(value).reduce(
          (acc, key) => ({
            ...acc,
            [key]: inflate(value[key]),
          }),
          {},
        ),
      };
    }
  } else {
    return {
      ...state,
      __newValue: state.__value,
    };
  }
}

const initialState: ConfigState = {
  configRoot: {
    __type: 'custom',
    __value: {},
  },
};

const configReducer = (
  state: ConfigState = initialState,
  action:
    | ReceiveConfigAction
    | UpdateConfigAction
    | SaveConfigAction
    | RefreshConfigAction,
): ConfigState => {
  switch (action.type) {
    case 'RECEIVE_CONFIG':
      return {
        ...state,
        configRoot: mergeModified(state.configRoot, action.configRoot),
      };
    case 'UPDATE_CONFIG':
      return {
        ...state,
        configRoot: action.configRoot,
      };
    case 'REFRESH_CONFIG':
      return {
        ...state,
        configRoot: revertModified(state.configRoot),
      };
    default:
      return state;
  }
};

export default configReducer;
