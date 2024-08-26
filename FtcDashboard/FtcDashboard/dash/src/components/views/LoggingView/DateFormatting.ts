export function DateToHHMMSS(date: Date): string {
  const h = `0${date.getHours()}`.slice(-2);
  const m = `0${date.getMinutes()}`.slice(-2);
  const s = `0${date.getSeconds()}`.slice(-2);
  const ms = `${date.getMilliseconds()}000`.slice(0, 3);

  return `${h}:${m}:${s}.${ms}`;
}
