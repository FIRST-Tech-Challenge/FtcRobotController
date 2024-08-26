import { CameraState, ReceiveImageAction, RECEIVE_IMAGE } from '@/store/types';

const initialState: CameraState = {
  imageStr: '',
};

const cameraReducer = (
  state: CameraState = initialState,
  action: ReceiveImageAction,
): CameraState => {
  switch (action.type) {
    case RECEIVE_IMAGE:
      return {
        ...state,
        imageStr: action.imageString,
      };
    default:
      return state;
  }
};

export default cameraReducer;
