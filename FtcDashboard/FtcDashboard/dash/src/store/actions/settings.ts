import { LayoutPresetType } from '@/enums/LayoutPreset';
import {
  GetLayoutPresetAction,
  ReceiveLayoutPresetAction,
  SaveLayoutPresetAction,
  GET_LAYOUT_PRESET,
  RECEIVE_LAYOUT_PRESET,
  SAVE_LAYOUT_PRESET,
} from '@/store/types';

export const saveLayoutPreset = (
  preset: LayoutPresetType,
): SaveLayoutPresetAction => ({
  type: SAVE_LAYOUT_PRESET,
  preset,
});

export const receiveLayoutPreset = (
  preset: LayoutPresetType,
): ReceiveLayoutPresetAction => ({
  type: RECEIVE_LAYOUT_PRESET,
  preset,
});

export const getLayoutPreset = (): GetLayoutPresetAction => ({
  type: GET_LAYOUT_PRESET,
});
