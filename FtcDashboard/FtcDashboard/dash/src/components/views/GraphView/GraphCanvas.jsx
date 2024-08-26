import React from 'react';
import PropTypes from 'prop-types';

import Graph from './Graph';
import AutoFitCanvas from '@/components/Canvas/AutoFitCanvas';
import { isEqual } from 'lodash';

class GraphCanvas extends React.Component {
  constructor(props) {
    super(props);

    this.canvasRef = React.createRef();

    this.renderGraph = this.renderGraph.bind(this);

    this.unsubs = []; // unsub functions to be called to cleanup

    this.state = {
      graphEmpty: false,
    };
  }

  componentDidMount() {
    this.graph = new Graph(this.canvasRef.current, this.props.options);
  }

  componentWillUnmount() {
    if (this.requestId) {
      cancelAnimationFrame(this.requestId);
    }
  }

  // TODO: Regretably, the current design requires that this.graph.add() only be called
  // once for each batch of telemetry. Violations of this contract cause artifacts in the
  // graph from out-of-order samples. (The graph code could be made more robust here, but
  // mitigating the issue here works just as well.)
  componentDidUpdate(prevProps) {
    let graphIsDirty = false;

    if (!isEqual(this.props.options, prevProps.options)) {
      this.graph.setOptions({
        ...this.graph.getOptions(),
        ...this.props.options,
      });
      graphIsDirty = true;
    }

    if (prevProps.paused && !this.props.paused) {
      this.graph.reset();
    }

    if (!this.props.paused && !isEqual(this.props.data, prevProps.data)) {
      this.graph.add(Date.now(), this.props.data);
    }

    if (!this.props.paused && !this.requestId) graphIsDirty = true;

    if (graphIsDirty) this.renderGraph();
  }

  renderGraph() {
    if (this.props.paused) {
      this.requestId = 0;
    } else {
      this.setState(() => ({
        graphEmpty: !this.graph.render(Date.now()),
      }));

      this.requestId = requestAnimationFrame(this.renderGraph);
    }
  }

  render() {
    return (
      <div className="flex-center h-full">
        <div
          className={`${this.state.graphEmpty ? 'hidden' : ''} h-full w-full`}
        >
          <AutoFitCanvas
            ref={this.canvasRef}
            onResize={() => {
              if (this.graph && this.props.paused)
                this.graph.render(this.props.pausedTime);
            }}
          />
        </div>
        <div className="flex-center pointer-events-none absolute top-0 left-0 h-full w-full">
          {this.state.graphEmpty && (
            <p className="text-center">No content to graph</p>
          )}
        </div>
      </div>
    );
  }
}

GraphCanvas.propTypes = {
  data: PropTypes.arrayOf(PropTypes.any).isRequired,
  options: PropTypes.object.isRequired,
  paused: PropTypes.bool.isRequired,
  pausedTime: PropTypes.number.isRequired,
};

export default GraphCanvas;
