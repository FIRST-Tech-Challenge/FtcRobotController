import { cloneDeep } from 'lodash';

type Options = {
  windowMs: number;
  colors: string[];
  lineWidth: number;
  padding: number;
  keySpacing: number;
  keyLineLength: number;
  gridLineWidth: number; // device pixels
  gridLineColor: string;
  fontSize: number;
  textColor: string;
  maxTicks: number;
};

import twColors from 'tailwindcss/colors';

// all dimensions in this file are *CSS* pixels unless otherwise stated
export const DEFAULT_OPTIONS: Options = {
  windowMs: 5000,
  colors: [
    twColors['blue']['600'],
    twColors['red']['600'],
    twColors['green']['600'],
    twColors['purple']['600'],
    twColors['orange']['600'],
    twColors['pink']['600'],
  ],
  lineWidth: 2,
  padding: 15,
  keySpacing: 4,
  keyLineLength: 12,
  gridLineWidth: 1, // device pixels
  gridLineColor: 'rgb(120, 120, 120)',
  fontSize: 14,
  textColor: 'rgb(50, 50, 50)',
  maxTicks: 7,
};

function niceNum(range: number, round: boolean) {
  const exponent = Math.floor(Math.log10(range));
  const fraction = range / Math.pow(10, exponent);
  let niceFraction;
  if (round) {
    if (fraction < 1.5) {
      niceFraction = 1;
    } else if (fraction < 3) {
      niceFraction = 2;
    } else if (fraction < 7) {
      niceFraction = 5;
    } else {
      niceFraction = 10;
    }
  } else if (fraction <= 1) {
    niceFraction = 1;
  } else if (fraction <= 2) {
    niceFraction = 2;
  } else if (fraction <= 5) {
    niceFraction = 5;
  } else {
    niceFraction = 10;
  }
  return niceFraction * Math.pow(10, exponent);
}

// interesting algorithm (see http://erison.blogspot.nl/2011/07/algorithm-for-optimal-scaling-on-chart.html)
function getAxisScaling(min: number, max: number, maxTicks: number) {
  const range = niceNum(max - min, false);
  const tickSpacing = niceNum(range / (maxTicks - 1), true);
  const niceMin = Math.floor(min / tickSpacing) * tickSpacing;
  const niceMax = (Math.floor(max / tickSpacing) + 1) * tickSpacing;
  return {
    min: niceMin,
    max: niceMax,
    spacing: tickSpacing,
  };
}

// shamelessly stolen from https://github.com/chartjs/Chart.js/blob/master/src/core/core.ticks.js
function formatTicks(tickValue: number, ticks: number[]) {
  // If we have lots of ticks, don't use the ones
  let delta = ticks.length > 3 ? ticks[2] - ticks[1] : ticks[1] - ticks[0];

  // If we have a number like 2.5 as the delta, figure out how many decimal places we need
  if (Math.abs(delta) > 1) {
    if (tickValue !== Math.floor(tickValue)) {
      // not an integer
      delta = tickValue - Math.floor(tickValue);
    }
  }

  const logDelta = Math.log10(Math.abs(delta));
  let tickString = '';

  if (tickValue !== 0) {
    let numDecimal = -1 * Math.floor(logDelta);
    numDecimal = Math.max(Math.min(numDecimal, 20), 0); // toFixed has a max of 20 decimal places
    tickString = tickValue.toFixed(numDecimal);
  } else {
    tickString = '0'; // never show decimal places for 0
  }

  return tickString;
}

type Axis = {
  min: number;
  max: number;
  spacing: number;
};

function getTicks(axis: Axis) {
  // get tick array
  const ticks = [];
  for (let i = axis.min; i <= axis.max; i += axis.spacing) {
    ticks.push(i);
  }

  // generate strings
  const tickStrings = [];
  for (let i = 0; i < ticks.length; i++) {
    const s = formatTicks(ticks[i], ticks);
    tickStrings.push(s);
  }

  return tickStrings;
}

function scale(
  value: number,
  fromLow: number,
  fromHigh: number,
  toLow: number,
  toHigh: number,
) {
  const frac = (toHigh - toLow) / (fromHigh - fromLow);
  return toLow + frac * (value - fromLow);
}

type Sample = {
  name: string;
  value: number;
};

// align coordinate to the nearest pixel, offset by a half pixel
// this helps with drawing thin lines; e.g., if a line of width 1px
// is drawn on an integer coordinate, it will be 2px wide
// x is assumed to be in *device* pixels
function alignCoord(x: number, scaling: number) {
  const roundX = Math.round(x * scaling);
  return (roundX + 0.5 * Math.sign(x - roundX)) / scaling;
}

type Scaling = {
  scalingX: number;
  scalingY: number;
};

function getScalingFactors(ctx: CanvasRenderingContext2D): Scaling {
  let transform;
  if (typeof ctx.getTransform === 'function') {
    transform = ctx.getTransform();
  } else if (typeof (ctx as any).mozCurrentTransform !== 'undefined') {
    transform = window.DOMMatrix.fromFloat64Array(
      Float64Array.from((ctx as any).mozCurrentTransform),
    );
  } else {
    throw new Error('unable to find canvas transform');
  }

  const { a, b, c, d } = transform;
  const scalingX = Math.sqrt(a * a + c * c);
  const scalingY = Math.sqrt(b * b + d * d);

  return {
    scalingX,
    scalingY,
  };
}

function fineMoveTo(
  ctx: CanvasRenderingContext2D,
  s: Scaling,
  x: number,
  y: number,
) {
  ctx.moveTo(alignCoord(x, s.scalingX), alignCoord(y, s.scalingY));
}

function fineLineTo(
  ctx: CanvasRenderingContext2D,
  s: Scaling,
  x: number,
  y: number,
) {
  ctx.lineTo(alignCoord(x, s.scalingX), alignCoord(y, s.scalingY));
}

export default class Graph {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  options: Options;

  data: { [key: string]: { ts: number[]; vs: number[]; color: string } };

  beginGraphNowMs = Number.NaN; // in telemetry time
  beginRenderTimeMs = Number.NaN; // in browser time

  scaling: Scaling;

  constructor(canvas: HTMLCanvasElement, options: Options) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d')!;

    this.options = cloneDeep(DEFAULT_OPTIONS);
    Object.assign(this.options, options || {});

    this.data = {};

    this.scaling = {
      scalingX: 1,
      scalingY: 1,
    };

    this.reset();
  }

  reset() {
    this.data = {};

    this.beginGraphNowMs = Number.NaN; // in telemetry time
    this.beginRenderTimeMs = Number.NaN; // in browser time
  }

  add(time: number, samples: Sample[][]) {
    const o = this.options;

    for (const sample of samples) {
      const t = sample.reduce(
        (acc, { name, value }) => (name === 'time' ? value : acc),
        Number.NaN,
      );

      for (const series of sample) {
        const { name, value } = series;

        if (name === 'time') continue;

        if (isNaN(value)) continue;

        if (!Object.prototype.hasOwnProperty.call(this.data, name)) {
          this.data[name] = {
            ts: [],
            vs: [],
            color: o.colors[Object.keys(this.data).length % o.colors.length],
          };
        }

        const { ts, vs } = this.data[name];
        ts.push(t);
        vs.push(value);
      }
    }

    if (isNaN(this.beginGraphNowMs) && samples.length > 0) {
      const maxT = samples[samples.length - 1].reduce(
        (acc, { name, value }) => (name === 'time' ? value : acc),
        Number.NaN,
      );
      this.beginGraphNowMs = maxT - 250; // introduce lag to allow for transmission time
      this.beginRenderTimeMs = time;
    }
  }

  getYAxisScaling() {
    const [min, max] = Object.keys(this.data).reduce(
      (acc, k) =>
        this.data[k].vs.reduce(
          ([min, max], v) => [Math.min(v, min), Math.max(v, max)],
          acc,
        ),
      [Number.MAX_VALUE, Number.MIN_VALUE],
    );

    if (Math.abs(min - max) < 1e-6) {
      return getAxisScaling(min - 1, max + 1, this.options.maxTicks);
    }

    return getAxisScaling(min, max, this.options.maxTicks);
  }

  render(time: number) {
    const o = this.options;

    // eslint-disable-next-line
    this.canvas.width = this.canvas.width; // clears the canvas

    if (isNaN(this.beginGraphNowMs)) return false;

    const graphNowMs = this.beginGraphNowMs + (time - this.beginRenderTimeMs);

    // prune old samples
    for (const k of Object.keys(this.data)) {
      const { ts, vs } = this.data[k];
      while (ts.length > 0 && ts[0] + o.windowMs + 250 < graphNowMs) {
        ts.shift();
        vs.shift();
      }
    }

    let allEmpty = true;
    for (const { ts } of Object.values(this.data)) {
      if (ts.length === 0) continue;

      allEmpty = false;
      break;
    }
    if (allEmpty) return false;

    // scale the canvas to facilitate the use of CSS pixels
    this.ctx.scale(devicePixelRatio, devicePixelRatio);

    this.scaling = getScalingFactors(this.ctx);

    this.ctx.font = `${o.fontSize}px "Roboto", sans-serif`;
    this.ctx.textBaseline = 'middle';
    this.ctx.textAlign = 'left';
    this.ctx.lineWidth = o.lineWidth / devicePixelRatio;

    const width = this.canvas.width / devicePixelRatio;
    const height = this.canvas.height / devicePixelRatio;

    const keyHeight = this.renderKey(0, 0, width);
    this.renderGraph(0, keyHeight, width, height - keyHeight, graphNowMs);

    return true;
  }

  renderKey(x: number, y: number, width: number) {
    const o = this.options;

    this.ctx.save();

    const names = Object.keys(this.data);
    const numSets = names.length;
    const height = numSets * o.fontSize + (numSets - 1) * o.keySpacing;
    for (let i = 0; i < numSets; i++) {
      const lineY = y + i * (o.fontSize + o.keySpacing) + o.fontSize / 2;
      const name = names[i];
      const { color } = this.data[name];
      const lineWidth =
        this.ctx.measureText(name).width + o.keyLineLength + o.keySpacing;
      const lineX = x + (width - lineWidth) / 2;

      this.ctx.strokeStyle = color;
      this.ctx.beginPath();
      fineMoveTo(this.ctx, this.scaling, lineX, lineY);
      fineLineTo(this.ctx, this.scaling, lineX + o.keyLineLength, lineY);
      this.ctx.stroke();

      this.ctx.fillStyle = o.textColor;
      this.ctx.fillText(name, lineX + o.keyLineLength + o.keySpacing, lineY);
    }

    this.ctx.restore();

    return height;
  }

  renderGraph(
    x: number,
    y: number,
    width: number,
    height: number,
    graphNowMs: number,
  ) {
    const o = this.options;

    const graphHeight = height - 2 * o.padding;

    const axis = this.getYAxisScaling();
    const ticks = getTicks(axis);
    const axisWidth = this.renderAxisLabels(
      x + o.padding,
      y + o.padding,
      graphHeight,
      ticks,
    );

    const graphWidth = width - axisWidth - 3 * o.padding;

    this.renderGridLines(
      x + axisWidth + 2 * o.padding,
      y + o.padding,
      graphWidth,
      graphHeight,
      5,
      ticks.length,
    );

    this.renderGraphLines(
      x + axisWidth + 2 * o.padding,
      y + o.padding,
      graphWidth,
      graphHeight,
      axis,
      graphNowMs,
    );
  }

  renderAxisLabels(x: number, y: number, height: number, ticks: string[]) {
    this.ctx.save();

    let width = 0;
    for (let i = 0; i < ticks.length; i++) {
      const textWidth = this.ctx.measureText(ticks[i]).width;
      if (textWidth > width) {
        width = textWidth;
      }
    }

    // draw axis labels
    this.ctx.textAlign = 'right';
    this.ctx.fillStyle = this.options.textColor;

    const vertSpacing = height / (ticks.length - 1);
    x += width;
    for (let i = 0; i < ticks.length; i++) {
      this.ctx.fillText(ticks[i], x, y + (ticks.length - i - 1) * vertSpacing);
    }

    this.ctx.restore();

    return width;
  }

  renderGridLines(
    x: number,
    y: number,
    width: number,
    height: number,
    numTicksX: number,
    numTicksY: number,
  ) {
    this.ctx.save();

    this.ctx.strokeStyle = this.options.gridLineColor;
    this.ctx.lineWidth = this.options.gridLineWidth / devicePixelRatio;

    const horSpacing = width / (numTicksX - 1);
    const vertSpacing = height / (numTicksY - 1);

    for (let i = 0; i < numTicksX; i++) {
      const lineX = x + horSpacing * i;
      this.ctx.beginPath();
      fineMoveTo(this.ctx, this.scaling, lineX, y);
      fineLineTo(this.ctx, this.scaling, lineX, y + height);
      this.ctx.stroke();
    }

    for (let i = 0; i < numTicksY; i++) {
      const lineY = y + vertSpacing * i;
      this.ctx.beginPath();
      fineLineTo(this.ctx, this.scaling, x, lineY);
      fineLineTo(this.ctx, this.scaling, x + width, lineY);
      this.ctx.stroke();
    }

    this.ctx.restore();
  }

  renderGraphLines(
    x: number,
    y: number,
    width: number,
    height: number,
    axis: Axis,
    graphNowMs: number,
  ) {
    const o = this.options;

    this.ctx.lineWidth = o.lineWidth;

    this.ctx.save();
    this.ctx.translate(x, y);
    this.ctx.rect(0, 0, width, height);
    this.ctx.clip();

    // draw data lines
    // scaling is used instead of transform because of the non-uniform stretching warps the plot line
    this.ctx.beginPath();
    Object.keys(this.data).forEach((k, i) => {
      const { ts, vs } = this.data[k];

      if (ts.length === 0) return;

      const color = o.colors[i % o.colors.length];

      this.ctx.beginPath();
      this.ctx.strokeStyle = color;
      fineMoveTo(
        this.ctx,
        this.scaling,
        scale(ts[0] - graphNowMs + o.windowMs, 0, o.windowMs, 0, width),
        scale(vs[0], axis.min, axis.max, height, 0),
      );
      for (let j = 1; j < ts.length; j++) {
        fineLineTo(
          this.ctx,
          this.scaling,
          scale(ts[j] - graphNowMs + o.windowMs, 0, o.windowMs, 0, width),
          scale(vs[j], axis.min, axis.max, height, 0),
        );
      }
      this.ctx.stroke();
    });

    this.ctx.restore();
  }

  getOptions() {
    return this.options;
  }

  setOptions(options: Options) {
    Object.assign(this.options, options);
  }
}
