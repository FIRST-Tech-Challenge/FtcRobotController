import { Values } from '@/typeHelpers';

import LayoutPreset from '@/enums/LayoutPreset';

export const SAVE_LAYOUT_PRESET = 'SAVE_LAYOUT_PRESET';
export const RECEIVE_LAYOUT_PRESET = 'RECEIVE_LAYOUT_PRESET';
export const GET_LAYOUT_PRESET = 'GET_LAYOUT_PRESET';

export type SettingState = {
  layoutPreset: Values<typeof LayoutPreset>;
};

export type SaveLayoutPresetAction = {
  type: typeof SAVE_LAYOUT_PRESET;
  preset: Values<typeof LayoutPreset>;
};

export type ReceiveLayoutPresetAction = {
  type: typeof RECEIVE_LAYOUT_PRESET;
  preset: Values<typeof LayoutPreset>;
};

export type GetLayoutPresetAction = {
  type: typeof GET_LAYOUT_PRESET;
};
