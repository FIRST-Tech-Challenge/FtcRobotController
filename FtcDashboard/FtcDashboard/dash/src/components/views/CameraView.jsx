import React from 'react';
import { connect } from 'react-redux';
import PropTypes from 'prop-types';

import AutoFitCanvas from '@/components/Canvas/AutoFitCanvas';
import { ReactComponent as RefreshIcon } from '@/assets/icons/refresh.svg';
import BaseView, {
  BaseViewHeading,
  BaseViewBody,
  BaseViewIcons,
  BaseViewIconButton,
} from './BaseView';

class CameraView extends React.Component {
  constructor(props) {
    super(props);

    this.canvasRef = React.createRef();

    this.renderImage = this.renderImage.bind(this);

    this.image = new Image();
    this.image.onload = this.renderImage;

    this.state = {
      rotation: 0,
    };
  }

  componentDidMount() {
    this.ctx = this.canvasRef.current.getContext('2d');
  }

  componentDidUpdate() {
    this.image.src = `data:image/jpeg;base64,${this.props.imageStr}`;
  }

  renderImage() {
    if (this.ctx && this.props.imageStr.length > 0) {
      const canvas = this.canvasRef.current;

      // eslint-disable-next-line
      canvas.width = canvas.width; // clears the canvas

      const viewportWidth = canvas.width;
      const viewportHeight = canvas.height;

      // rotate the image
      const scale = Math.min(
        devicePixelRatio,
        (this.state.rotation % 2 === 0 ? viewportHeight : viewportWidth) /
          this.image.height,
        (this.state.rotation % 2 === 0 ? viewportWidth : viewportHeight) /
          this.image.width,
      );
      this.ctx.translate(viewportWidth / 2, viewportHeight / 2);
      this.ctx.rotate((this.state.rotation * Math.PI) / 2);
      this.ctx.scale(scale, scale);
      this.ctx.drawImage(
        this.image,
        -this.image.width / 2,
        -this.image.height / 2,
        this.image.width,
        this.image.height,
      );
    }
  }

  render() {
    return (
      <BaseView isUnlocked={this.props.isUnlocked}>
        <div className="flex">
          <BaseViewHeading isDraggable={this.props.isDraggable}>
            Camera
          </BaseViewHeading>
          <BaseViewIcons>
            <BaseViewIconButton title="Rotate">
              <RefreshIcon
                className="h-6 w-6"
                onClick={() =>
                  this.setState({ rotation: (this.state.rotation + 1) % 4 })
                }
              />
            </BaseViewIconButton>
          </BaseViewIcons>
        </div>
        <BaseViewBody>
          <div style={{ height: '100%', minHeight: '10rem' }}>
            <AutoFitCanvas ref={this.canvasRef} onResize={this.renderImage} />
          </div>
        </BaseViewBody>
      </BaseView>
    );
  }
}

CameraView.propTypes = {
  imageStr: PropTypes.string.isRequired,

  isDraggable: PropTypes.bool,
  isUnlocked: PropTypes.bool,
};

const mapStateToProps = ({ camera }) => ({
  imageStr: camera.imageStr,
});

export default connect(mapStateToProps)(CameraView);
