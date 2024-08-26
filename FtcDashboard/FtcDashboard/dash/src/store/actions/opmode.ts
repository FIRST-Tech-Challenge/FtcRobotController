import {
  InitOpModeAction,
  INIT_OP_MODE,
  StartOpModeAction,
  START_OP_MODE,
  StopOpModeAction,
  STOP_OP_MODE,
} from '@/store/types';

export const initOpMode = (opModeName: string): InitOpModeAction => ({
  type: INIT_OP_MODE,
  opModeName,
});

export const startOpMode = (): StartOpModeAction => ({
  type: START_OP_MODE,
});

export const stopOpMode = (): StopOpModeAction => ({
  type: STOP_OP_MODE,
});
