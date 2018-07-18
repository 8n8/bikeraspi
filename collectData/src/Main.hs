module Main where

import Graphics.Webcam.Linux

main :: IO ()
main = runCam (Webcam 0) $
    grab >>= saveBmp "1.bmp"

-- main = do
--   putStrLn "hello world"
