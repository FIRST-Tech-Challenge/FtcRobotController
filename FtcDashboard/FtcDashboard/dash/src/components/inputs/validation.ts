export type ValResult<T> =
  | {
      value: T;
      valid: true;
    }
  | {
      value: string;
      valid: false;
    };

export function validateDouble(value: string): ValResult<number> {
  if (isFinite(Number(value)) && !/^\s*$/.test(value)) {
    return {
      value: Number(value),
      valid: true,
    };
  } else {
    return {
      value,
      valid: false,
    };
  }
}

export function validateInt(value: string): ValResult<number> {
  if (/^-?\d+$/.test(value)) {
    return {
      value: parseInt(value, 10),
      valid: true,
    };
  } else {
    return {
      value,
      valid: false,
    };
  }
}

export function validateString(value: string): ValResult<string> {
  return {
    value,
    valid: true,
  };
}
