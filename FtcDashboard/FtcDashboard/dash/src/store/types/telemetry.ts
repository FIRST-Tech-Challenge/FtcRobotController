export const RECEIVE_TELEMETRY = 'RECEIVE_TELEMETRY';

export type Telemetry = TelemetryItem[];

type Fill = {
  type: 'fill';
  color: string;
};

type Stroke = {
  type: 'stroke';
  color: string;
};

type StrokeWidth = {
  type: 'strokeWidth';
  lineWidth: number;
};

type Circle = {
  type: 'circle';
  x: number;
  y: number;
  radius: number;
};

type Polygon = {
  type: 'polygon';
  xPoints: number[];
  yPoints: number[];
  stroke: string;
};

type Polyline = {
  type: 'polyline';
  xPoints: number[];
  yPoints: number[];
};

type Spline = {
  type: 'spline';
  ax: number;
  bx: number;
  cx: number;
  dx: number;
  ex: number;
  fx: number;
  ay: number;
  by: number;
  cy: number;
  dy: number;
  ey: number;
  fy: number;
};

type DrawOp =
  | Fill
  | Stroke
  | StrokeWidth
  | Circle
  | Polygon
  | Polyline
  | Spline;

export type TelemetryItem = {
  data: {
    [key: string]: string;
  };

  field: {
    ops: DrawOp[];
  };
  fieldOverlay: {
    ops: DrawOp[];
  };
  log: string[];
  timestamp: number;
};

export type ReceiveTelemetryAction = {
  type: typeof RECEIVE_TELEMETRY;
  telemetry: Telemetry;
};
