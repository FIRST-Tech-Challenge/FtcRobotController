import { Telemetry, RECEIVE_TELEMETRY } from '@/store/types';

export const receiveTelemetry = (telemetry: Telemetry) => ({
  type: RECEIVE_TELEMETRY,
  telemetry,
});
