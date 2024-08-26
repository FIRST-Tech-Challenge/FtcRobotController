export const CONNECT = 'CONNECT';
export const DISCONNECT = 'DISCONNECT';
export const RECEIVE_PING_TIME = 'RECEIVE_PING_TIME';
export const RECEIVE_CONNECTION_STATUS = 'RECEIVE_CONNECTION_STATUS';
export const SEND_MESSAGE = 'SEND_MESSAGE';

export type SocketState = {
  isConnected: boolean;
  pingTime: number;
};

export type ConnectAction = {
  type: typeof CONNECT;
  host: string;
  port: string;
};

export type DisconnectAction = {
  type: typeof DISCONNECT;
};

export type ReceivePingTimeAction = {
  type: typeof RECEIVE_PING_TIME;
  pingTime: number;
};

export type ReceiveConnectionStatusAction = {
  type: typeof RECEIVE_CONNECTION_STATUS;
  isConnected: boolean;
};
