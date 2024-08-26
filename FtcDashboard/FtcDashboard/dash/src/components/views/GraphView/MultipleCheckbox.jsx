import React from 'react';
import PropTypes from 'prop-types';
import { v4 as uuid4 } from 'uuid';

class MultipleCheckbox extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      selected: this.props.selected || [],
      uuid: uuid4(),
    };
  }

  handleChange(evt, val) {
    if (evt.target.checked) {
      this.setState(
        {
          selected: [...this.state.selected, val],
        },
        () => this.props.onChange(this.state.selected),
      );
    } else {
      this.setState(
        {
          selected: this.state.selected.filter((el) => val !== el),
        },
        () => this.props.onChange(this.state.selected),
      );
    }
  }

  render() {
    return (
      <table className="overflow-y-scroll">
        <tbody>
          {this.props.arr
            .filter(
              (val) =>
                !this.props.exclude || this.props.exclude.indexOf(val) === -1,
            )
            .map((val) => (
              <tr key={val}>
                <td>
                  <input
                    id={`multiple-checkbox-${this.state.uuid}-${val}`}
                    className="rounded text-primary-600 transition hover:border-gray-900 hover:shadow focus:ring-primary-600 dark:ring-offset-slate-100/40"
                    type="checkbox"
                    onChange={(evt) => this.handleChange(evt, val)}
                    checked={this.state.selected.indexOf(val) !== -1}
                  />
                </td>
                <td>
                  <label
                    htmlFor={`multiple-checkbox-${this.state.uuid}-${val}`}
                  >
                    {val}
                  </label>
                </td>
              </tr>
            ))}
        </tbody>
      </table>
    );
  }
}

MultipleCheckbox.propTypes = {
  arr: PropTypes.arrayOf(PropTypes.string).isRequired,
  selected: PropTypes.arrayOf(PropTypes.string),
  exclude: PropTypes.arrayOf(PropTypes.string),
  onChange: PropTypes.func, // TODO: fix!
};

export default MultipleCheckbox;
