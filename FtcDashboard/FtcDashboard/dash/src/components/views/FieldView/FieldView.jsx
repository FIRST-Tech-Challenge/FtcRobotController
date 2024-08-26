import React from 'react';
import PropTypes from 'prop-types';
import { connect } from 'react-redux';

import BaseView, { BaseViewHeading } from '@/components/views/BaseView';
import Field from './Field';
import AutoFitCanvas from '@/components/Canvas/AutoFitCanvas';

class FieldView extends React.Component {
  constructor(props) {
    super(props);

    this.canvasRef = React.createRef();

    this.renderField = this.renderField.bind(this);

    this.overlay = {
      ops: [],
    };
  }

  componentDidMount() {
    this.field = new Field(this.canvasRef.current);
    this.renderField();
  }

  componentDidUpdate(prevProps) {
    if (this.props.telemetry === prevProps.telemetry) return;

    this.overlay = this.props.telemetry.reduce(
      (acc, { field, fieldOverlay }) =>
        fieldOverlay.ops.length === 0
          ? acc
          : {
              ops: [...field.ops, ...fieldOverlay.ops],
            },
      this.overlay,
    );

    this.field.setOverlay(this.overlay);
    this.renderField();
  }

  renderField() {
    if (this.field) {
      this.field.render();
    }
  }

  render() {
    return (
      <BaseView isUnlocked={this.props.isUnlocked}>
        <BaseViewHeading isDraggable={this.props.isDraggable}>
          Field
        </BaseViewHeading>
        <AutoFitCanvas
          ref={this.canvasRef}
          onResize={this.renderField}
          containerHeight="calc(100% - 3em)"
        />
      </BaseView>
    );
  }
}

FieldView.propTypes = {
  telemetry: PropTypes.arrayOf(PropTypes.object).isRequired,
  isDraggable: PropTypes.bool,
  isUnlocked: PropTypes.bool,
};

const mapStateToProps = ({ telemetry }) => ({
  telemetry,
});

export default connect(mapStateToProps)(FieldView);
