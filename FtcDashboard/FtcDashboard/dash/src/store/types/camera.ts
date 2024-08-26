export const RECEIVE_IMAGE = 'RECEIVE_IMAGE';

export type CameraState = {
  imageStr: string;
};

export type ReceiveImageAction = {
  type: typeof RECEIVE_IMAGE;
  imageString: string;
};
