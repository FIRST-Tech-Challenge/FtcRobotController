export const INIT_OP_MODE = 'INIT_OP_MODE';
export const START_OP_MODE = 'START_OP_MODE';
export const STOP_OP_MODE = 'STOP_OP_MODE';

export const STOP_OP_MODE_TAG = '$Stop$Robot$';

export type InitOpModeAction = {
  type: typeof INIT_OP_MODE;
  opModeName: string;
};

export type StartOpModeAction = {
  type: typeof START_OP_MODE;
};

export type StopOpModeAction = {
  type: typeof STOP_OP_MODE;
};
