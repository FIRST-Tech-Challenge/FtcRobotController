import PropTypes from 'prop-types';

const Tile = ({ children, row, col }) => (
  <div
    className="border border-gray-100 transition-colors dark:border-slate-800"
    style={{
      gridRow: row,
      gridColumn: col,
    }}
  >
    {children}
  </div>
);

Tile.propTypes = {
  children: PropTypes.node.isRequired,
  row: PropTypes.oneOfType([PropTypes.number, PropTypes.string]).isRequired,
  col: PropTypes.oneOfType([PropTypes.number, PropTypes.string]).isRequired,
};

export default Tile;
