import { ReceiveImageAction, RECEIVE_IMAGE } from '@/store/types';

export const receiveImage = (imageString: string): ReceiveImageAction => ({
  type: RECEIVE_IMAGE,
  imageString,
});
