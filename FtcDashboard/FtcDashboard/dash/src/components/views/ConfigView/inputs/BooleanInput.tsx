import React from 'react';

interface Props {
  id: string;
  value: boolean;
  onChange: (arg: { value: boolean; valid: boolean }) => void;
  onSave: () => void;
}

const BooleanInput: React.FC<Props> = ({ id, value, onChange, onSave }) => (
  <span className="flex items-center">
    <input
      id={id}
      className="mr-4 rounded text-primary-600 transition hover:border-gray-900 hover:shadow focus:ring-primary-600 dark:ring-offset-slate-100/40"
      type="checkbox"
      checked={value}
      onChange={(evt) =>
        onChange({
          value: evt.target.checked,
          valid: true,
        })
      }
    />
    <button
      className={`
        rounded border border-gray-200 bg-gray-100 px-2 transition
        hover:border-gray-400 hover:shadow focus:border-gray-500 focus:bg-gray-300
         dark:border-transparent dark:bg-primary-600 dark:highlight-primary-100/30 dark:hover:border-primary-400/80 dark:hover:shadow-md dark:hover:shadow-blue-200/20 dark:focus:bg-primary-700
        `}
      onClick={onSave}
    >
      Save
    </button>
  </span>
);

export default BooleanInput;
