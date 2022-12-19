// // noinspection JSCheckFunctionSignatures
//
// import { createStyles, keyframes } from "@mantine/core";
// import { ReactComponent as BackgroundDesign } from "../../assets/sentinel-website-background-thingy-v2.svg";
// import { GradientBackground } from "./GradientBackground";
//
// export function GradientBackgroundWithLineDesign({ className, designClassName, fixed, ...props }) {
//   const { classes, cx } = useStyles();
//
//   return (
//     <GradientBackground>
//       <div className={cx(classes.designWrapper, className)} style={{ position: fixed ? 'fixed' : 'absolute' }}>
//         <BackgroundDesign className={cx(classes.backgroundImage, designClassName)}/>
//       </div>
//       {props.children}
//     </GradientBackground>
//   );
// }
//
// const useStyles = createStyles((theme) => {
//   const dark = theme.colorScheme === 'dark';
//
//   return {
//     backgroundImage: {
//       ...theme.fn.cover(),
//       fill: theme.colors.gray[dark ? 4 : 2],
//       stroke: theme.colors.gray[dark ? 4 : 2],
//     },
//
//     designWrapper: {
//       top: 0,
//       right: 0,
//       height: '100vmin',
//       width: '100vmin',
//       animation: `${designAppear} 1s forwards`,
//       zIndex: -1,
//     },
//   }
// });
//
// const designAppear = keyframes((() => {
//   let _keyframes = {};
//   for (let i = 25; i >= 0; i--) {
//     _keyframes[`${100 - i * 4}%`] = {
//       maskImage: `linear-gradient(45deg, transparent 0%, black ${i * 48}%)`,
//     }
//   }
//   return _keyframes;
// })());