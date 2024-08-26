import {
  ReceiveTelemetryAction,
  RECEIVE_TELEMETRY,
  Telemetry,
} from '@/store/types';

const initialState: Telemetry = [
  {
    timestamp: 0,
    data: {},
    log: [],
    field: {
      ops: [],
    },
    fieldOverlay: {
      ops: [],
    },
  },
];

const telemetryReducer = (
  state = initialState,
  action: ReceiveTelemetryAction,
) => {
  switch (action.type) {
    case RECEIVE_TELEMETRY:
      return action.telemetry;
    default:
      return state;
  }
};

export default telemetryReducer;
