import React from 'react';
import PropTypes from 'prop-types';

class AutoFitCanvas extends React.Component {
  constructor(props) {
    super(props);

    this.resize = this.resize.bind(this);

    this.containerRef = React.createRef();
    this.resizeObserver = new ResizeObserver((entries) => {
      for (let entry of entries) {
        if (entry.target === this.containerRef.current) {
          this.resize();
        }
      }
    });
  }

  componentDidMount() {
    this.resize();

    this.resizeObserver.observe(this.containerRef.current);
  }

  componentWillUnmount() {
    this.resizeObserver.disconnect();
  }

  resize() {
    const canvas = this.props.innerRef.current;
    canvas.width = this.containerRef.current.clientWidth * devicePixelRatio;
    canvas.height = this.containerRef.current.clientHeight * devicePixelRatio;

    canvas.style.width = `${canvas.width / devicePixelRatio}px`;
    canvas.style.height = `${canvas.height / devicePixelRatio}px`;

    this.props.onResize?.();
  }

  render() {
    return (
      <div
        style={{ width: '100%', height: this.props.containerHeight }}
        ref={this.containerRef}
      >
        <canvas ref={this.props.innerRef} />
      </div>
    );
  }
}

AutoFitCanvas.defaultProps = {
  containerHeight: '100%',
};

AutoFitCanvas.propTypes = {
  innerRef: PropTypes.any.isRequired,
  onResize: PropTypes.func,
  containerHeight: PropTypes.string,
};

const ForwardedAutoFitCanvas = React.forwardRef((props, ref) => (
  <AutoFitCanvas innerRef={ref} {...props} />
));
ForwardedAutoFitCanvas.displayName = 'AutoFitCanvas';
export default ForwardedAutoFitCanvas;
