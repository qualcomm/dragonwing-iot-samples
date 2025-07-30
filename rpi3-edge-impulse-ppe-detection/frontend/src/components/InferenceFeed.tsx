/* -----------------------------------------------------------------------------

  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
  SPDX-License-Identifier: BSD-3-Clause
 
   ----------------------------------------------------------------------------- */
   

import { useEffect, useState } from 'react'
import { PPEItem, type PPEItemInfo } from '../components/PPEItem'


interface ValueAndAverage {
  value: number,
  average: number,
}


export const InferenceFeed = ({ webSocketUrl }: { webSocketUrl: string }) => {
  const [PPEItems, setPPEItems] = useState<PPEItemInfo[]>([]);
  const [cameraSrc, setCameraSrc] = useState<string | null>(null);
  const [inferenceTime, setInferenceTime] = useState<ValueAndAverage | null>(null);
  const [inferenceFPS, setInferenceFPS] = useState<ValueAndAverage | null>(null);

  useEffect(() => {
    // Establish WebSocket connection
    const modelWs = new WebSocket(webSocketUrl);

    // Listen for messages from the server
    modelWs.onmessage = (event) => {
        const json_data = JSON.parse(event.data)
        setPPEItems(json_data.data);

        setCameraSrc(`data:image/jpeg;base64,${json_data.image}`);

        setInferenceTime(json_data.inference_time);
        setInferenceFPS(json_data.inference_fps);
    };

    // Cleanup on component unmount
    return () => {
        modelWs.close();

        setCameraSrc(null);
    }
}, []);

  let videoElement: React.ReactElement | null = null;
  if (cameraSrc !== null) {
    videoElement = <img id="camera-stream" alt="Camera Stream" className="img-stream" src={cameraSrc}></img>;
  } else {
    videoElement = <p>Camera is loading...</p>;
  }


  return (
    <div>        
        {videoElement}
        
        {
            PPEItems.map((ppeItem) => (
            <PPEItem itemInfo={ppeItem}></PPEItem>
            ))
        }
        <br></br>
        
        {inferenceTime && inferenceFPS && (
          <p>
            Current inference time: {(inferenceTime.value * 1000).toFixed(3)} ms 
            ({inferenceFPS.value.toFixed(3)} fps)<br></br>
            
            Average inference time: {(inferenceTime.average * 1000).toFixed(3)} ms 
            ({inferenceFPS.average.toFixed(3)} fps)
          </p>
        )}
    </div>
  )
}
