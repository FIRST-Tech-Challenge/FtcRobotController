import { useEffect, useState, useRef, RefObject } from 'react';

export default function useDelayedTooltip(
  delay: number,
  ref: RefObject<HTMLElement | null>,
) {
  const [isShowingTooltip, setIsShowingTooltip] = useState(false);
  const isMouseStillIn = useRef(false);

  const timeoutTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    const onMouseEnter = () => {
      isMouseStillIn.current = true;

      if (timeoutTimer.current !== null) clearInterval(timeoutTimer.current);

      timeoutTimer.current = setTimeout(() => {
        if (isMouseStillIn.current) {
          setIsShowingTooltip(true);
        } else {
          setIsShowingTooltip(false);
        }
      }, delay * 1000);
    };
    const onMouseLeave = () => {
      isMouseStillIn.current = false;
      setIsShowingTooltip(false);
    };

    if (ref.current !== null) {
      ref.current?.removeEventListener('mouseenter', onMouseEnter);
      ref.current?.removeEventListener('focusin', onMouseEnter);
      ref.current?.removeEventListener('mouseleave', onMouseLeave);
      ref.current?.removeEventListener('focusout', onMouseLeave);
    }

    if (ref.current) {
      ref.current.addEventListener('mouseenter', onMouseEnter);
      ref.current.addEventListener('focusin', onMouseEnter);
      ref.current.addEventListener('mouseleave', onMouseLeave);
      ref.current.addEventListener('focusout', onMouseLeave);
    }
  }, [delay, ref]);

  useEffect(() => {
    return () => {
      if (timeoutTimer.current !== null) clearTimeout(timeoutTimer.current);
    };
  }, []);

  return isShowingTooltip;
}
