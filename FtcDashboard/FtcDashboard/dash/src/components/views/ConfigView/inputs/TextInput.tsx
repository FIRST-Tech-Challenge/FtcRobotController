import React from 'react';
import clsx from 'clsx';
import { ValResult } from '@/components/inputs/validation';

interface Props<T> {
  id?: string;
  value: string | number;
  valid: boolean;
  validate: (rawValue: string) => ValResult<T>;
  onChange: (arg: ValResult<T>) => void;
  onSave?: () => void;
}

const TextInput = <T,>({
  id,
  value,
  valid,
  validate,
  onChange,
  onSave,
}: Props<T>) => {
  const [inputValue, setInputValue] = React.useState(`${value}`);

  const inputRef = React.useRef<HTMLInputElement>(null);

  React.useEffect(() => {
    inputRef.current?.setCustomValidity(valid ? '' : 'Invalid input');
  }, [valid]);

  React.useEffect(() => {
    if (value !== validate(inputValue).value) {
      setInputValue(`${value}`);
    }
  }, [value, validate, inputValue]);

  const handleChange = (evt: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(evt.target.value);
    const validated = validate(evt.target.value);
    if (validated) {
      onChange(validated);
    }
  };

  const handleKeyDown = (evt: React.KeyboardEvent<HTMLInputElement>) => {
    if (onSave && evt.keyCode === 13) onSave();
  };

  return (
    <input
      id={id}
      className={clsx(
        'rounded border border-gray-200 bg-gray-100 px-3 py-1 transition focus:border-primary-500 focus:ring-primary-500',
        'dark:border-slate-500/80 dark:bg-slate-700 dark:text-slate-200',
        !valid && 'border-red-500 focus:border-red-500 focus:ring-red-500',
      )}
      ref={inputRef}
      type="text"
      size={15}
      value={inputValue}
      onChange={handleChange}
      onKeyDown={handleKeyDown}
    />
  );
};

export default TextInput;
