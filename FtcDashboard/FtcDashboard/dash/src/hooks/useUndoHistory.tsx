import { useReducer, useMemo } from 'react';
import { isEqual } from 'lodash';

enum StateHistoryCommand {
  INITIALIZE,
  APPEND,
  UNDO,
  REDO,
}

export default function useUndoHistory<T>(initialHead: T): [
  T,
  {
    initialize: (payload: T) => void;
    append: (payload: T) => void;
    undo: () => void;
    redo: () => void;
  },
] {
  type StateHistoryAction =
    | {
        type: StateHistoryCommand.INITIALIZE;
        payload: T;
      }
    | {
        type: StateHistoryCommand.APPEND;
        payload: T;
      }
    | { type: StateHistoryCommand.UNDO }
    | { type: StateHistoryCommand.REDO };

  type StateHistoryReducerState = {
    history: T[];
    currentHistoryPosition: number;
  };

  const stateHistoryReducer = (
    state: StateHistoryReducerState,
    action: StateHistoryAction,
  ): StateHistoryReducerState => {
    switch (action.type) {
      case StateHistoryCommand.INITIALIZE: {
        return {
          history: [action.payload],
          currentHistoryPosition: 0,
        };
      }
      case StateHistoryCommand.APPEND: {
        if (state.history.length !== 0) {
          if (
            isEqual(state.history[state.currentHistoryPosition], action.payload)
          ) {
            return state;
          }
        }

        return {
          history: [
            ...state.history.slice(0, state.currentHistoryPosition + 1),
            action.payload,
          ],
          currentHistoryPosition: state.currentHistoryPosition + 1,
        };
      }
      case StateHistoryCommand.UNDO: {
        if (state.currentHistoryPosition <= 0) return state;

        return {
          ...state,
          currentHistoryPosition: state.currentHistoryPosition - 1,
        };
      }
      case StateHistoryCommand.REDO: {
        if (state.currentHistoryPosition >= state.history.length - 1)
          return state;

        return {
          ...state,
          currentHistoryPosition: state.currentHistoryPosition + 1,
        };
      }
    }
  };

  const [{ history, currentHistoryPosition }, dispatch] = useReducer(
    stateHistoryReducer,
    {
      history: [initialHead],
      currentHistoryPosition: 0,
    },
  );

  // Requires a useMemo or it causes an infinite loop in useEffect dependencies
  const initialize = useMemo(
    () => (payload: T) =>
      dispatch({ type: StateHistoryCommand.INITIALIZE, payload }),
    [dispatch],
  );
  const append = useMemo(
    () => (payload: T) =>
      dispatch({ type: StateHistoryCommand.APPEND, payload }),
    [dispatch],
  );
  const undo = useMemo(
    () => () => dispatch({ type: StateHistoryCommand.UNDO }),
    [dispatch],
  );
  const redo = useMemo(
    () => () => dispatch({ type: StateHistoryCommand.REDO }),
    [dispatch],
  );

  return [history[currentHistoryPosition], { initialize, append, undo, redo }];
}
