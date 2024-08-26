import { useState, useEffect, useRef } from 'react';

import {
  GridChildComponentProps,
  GridOnScrollProps,
  VariableSizeGrid as Grid,
} from 'react-window';
import AutoSizer from 'react-virtualized-auto-sizer';
import { DraggableCore } from 'react-draggable';

type Props = {
  header: string[];
  data: unknown[];
  columnsShowing?: boolean[];
};

const Cell = ({
  columnIndex,
  rowIndex,
  style,
  data,
}: GridChildComponentProps) => {
  const dataTyped = data as { cellData: string[]; visibleColumns: boolean[] };

  return dataTyped.visibleColumns[columnIndex] ? (
    <div
      className={`truncate px-2 ${
        columnIndex === 0 ? 'text-neutral-gray-400' : ''
      }`}
      style={style}
    >
      {dataTyped.cellData[rowIndex][columnIndex]}
    </div>
  ) : null;
};

const DEFAULT_COL_WIDTH = 150;
const COL_MIN_WIDTH = 50;

const ROW_HEIGHT = 30;

const AUTO_SCROLL_BOTTOM_THRESHOLD = 15;

const CustomVirtualGrid = ({ header, data, columnsShowing }: Props) => {
  const [colWidth, setColWidth] = useState<number[]>([]);
  const [headerOffset, setHeaderOffset] = useState(0);

  const [isScrollAtBottom, setIsScrollAtBottom] = useState(true);

  const gridRef = useRef<Grid>(null);

  useEffect(() => {
    setColWidth(new Array(header.length).fill(DEFAULT_COL_WIDTH));
  }, [header.length]);

  useEffect(() => {
    if (data.length === 0) {
      setIsScrollAtBottom(true);
    }
  }, [data.length]);

  useEffect(() => {
    gridRef.current?.resetAfterColumnIndex(0);
  }, [colWidth, columnsShowing]);

  useEffect(() => {
    if (gridRef.current) {
      if (isScrollAtBottom) {
        gridRef.current.scrollToItem({
          rowIndex: data.length,
        });
      }
    }
  }, [data.length, headerOffset, isScrollAtBottom]);

  const resizeCol = (key: string, deltaX: number) => {
    const index = header.indexOf(key);

    if (index === -1) return;

    const colCopy = [...colWidth];
    colCopy[index] = Math.max(COL_MIN_WIDTH, colCopy[index] + deltaX);

    setColWidth(colCopy);
  };

  const onScroll = ({
    scrollLeft,
    scrollTop,
    scrollUpdateWasRequested,
  }: GridOnScrollProps) => {
    setHeaderOffset(scrollLeft);

    if (gridRef.current) {
      const bottom =
        gridRef.current.props.rowCount * ROW_HEIGHT -
        (gridRef.current.props.height as number) -
        scrollTop;

      if (!scrollUpdateWasRequested) {
        if (bottom <= AUTO_SCROLL_BOTTOM_THRESHOLD) setIsScrollAtBottom(true);
        else setIsScrollAtBottom(false);
      }
    }
  };

  const derivedColWidth = colWidth.map((e, i) =>
    (columnsShowing ?? [])[i] === false ? 0 : e,
  );

  return (
    <div className="flex h-full w-full flex-col overflow-x-hidden">
      <div
        className="mb-1"
        style={{
          width: (colWidth.length === 0 ? [0] : colWidth).reduce(
            (prev, acc) => prev + acc,
          ),
          transform: `translateX(${-headerOffset}px)`,
        }}
      >
        {header.map((e, i) => (
          <div
            key={e}
            className={`relative inline-flex flex-row pr-8 ${
              derivedColWidth[i] === 0 ? 'hidden' : ''
            }`}
            style={{ width: colWidth[i], minWidth: '3em' }}
          >
            <span className="flex-grow truncate font-semibold">{e}</span>
            <DraggableCore onDrag={(_, { deltaX }) => resizeCol(e, deltaX)}>
              <div className="absolute right-1 mr-2 cursor-col-resize rounded px-2 transition-colors hover:bg-gray-200 hover:bg-opacity-75">
                ⋮
              </div>
            </DraggableCore>
          </div>
        ))}
      </div>
      <div className="flex-grow">
        <AutoSizer>
          {({ height, width }) =>
            data.length > 0 ? (
              <Grid
                ref={gridRef}
                columnCount={header.length}
                columnWidth={(i) => derivedColWidth[i]}
                height={height}
                rowCount={data.length}
                rowHeight={() => ROW_HEIGHT}
                width={width}
                itemData={{
                  cellData: data,
                  visibleColumns: derivedColWidth.map((e) => e !== 0),
                }}
                onScroll={onScroll}
              >
                {Cell}
              </Grid>
            ) : (
              <div
                style={{ width, height }}
                className="flex-center text-center"
              >
                Logs will be recorded as an opmode starts streaming telemetry
                data
                <br />A CSV download will be available once the opmode is
                stopped
              </div>
            )
          }
        </AutoSizer>
      </div>
    </div>
  );
};

export default CustomVirtualGrid;
