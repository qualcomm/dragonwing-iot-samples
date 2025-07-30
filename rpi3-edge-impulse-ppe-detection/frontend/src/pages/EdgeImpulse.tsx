/* -----------------------------------------------------------------------------

  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
  SPDX-License-Identifier: BSD-3-Clause
 
   ----------------------------------------------------------------------------- */


import NavBar from '../components/NavBar'
import { InferenceFeed } from '../components/InferenceFeed';


function EdgeImpulsePage() {
  const hostname = window.location.hostname;  
  const webSocketUrl: string = `ws://${hostname}:8765/edge-impulse`;

  return (
    <div>
        <NavBar/>
        
        <div className="center-box">
        <h1>Edge Impulse Model</h1>

        <h2>Edge Impulse: AI for Any Edge Device</h2>
        <p>
          Edge Impulse, now a Qualcomm company, is the leading development platform for edge AI. 
          From microcontrollers to gateways, it empowers teams to build datasets, train models, and 
          deploy optimized AI directly on devices. Whether you're working with sensors, cameras, or 
          industrial machines, Edge Impulse simplifies embedded machine learning and accelerates 
          innovation across industries.
        </p>

        <h2>PPE Detection</h2>
        <p>
          Below is a custom model built on the Edge Impulse ecosystem.
        </p>
        
        <InferenceFeed webSocketUrl={webSocketUrl}></InferenceFeed>
        </div>
    </div>
  )
}

export default EdgeImpulsePage
