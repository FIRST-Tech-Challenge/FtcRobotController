import { useEffect, RefObject } from 'react';

export default function useOnClickOutside(
  ref: RefObject<HTMLElement | null>,
  handler: (event: TouchEvent | MouseEvent) => void,
  excludedRefs?: RefObject<HTMLElement | null>[],
) {
  useEffect(() => {
    const listener = (event: TouchEvent | MouseEvent) => {
      if (
        !ref.current ||
        ref.current.contains(event.target as Node) ||
        excludedRefs?.every((e) => e.current?.contains(event.target as Node))
      ) {
        return;
      }

      handler(event);
    };

    document.addEventListener('mousedown', listener);
    document.addEventListener('touchstart', listener);

    return () => {
      document.removeEventListener('mousedown', listener);
      document.removeEventListener('touchstart', listener);
    };
  }, [ref, handler, excludedRefs]);
}
