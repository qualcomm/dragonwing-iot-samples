/* -----------------------------------------------------------------------------

  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
  SPDX-License-Identifier: BSD-3-Clause
 
   ----------------------------------------------------------------------------- */

   
import NavBar from '../components/NavBar'
import { InferenceFeed } from '../components/InferenceFeed'

function QAIHubPage() {
  const hostname = window.location.hostname;  
  const webSocketUrl: string = `ws://${hostname}:8765/qai-hub`;

  return (
    <div>
        <NavBar/>
        
        <div className="center-box">
        <h1>QAI Hub Model</h1>
        <h2>Qualcomm AI Hub: The Platform for On-Device AI</h2>
        <p>
          Deploy any model to any device in minutes with Qualcomm AI Hub. Whether you're working with mobile, 
          automotive, IoT, or compute platforms, AI Hub streamlines the journey from model creation to on-device 
          deployment. Leverage a rich ecosystem of model makers, cloud services, runtimes, and SDKs to build end-to-end 
          ML solutions. Optimize, profile, and deploy with ease using tools like ONNX Runtime, TensorFlow Lite, and 
          Qualcomm AI Runtime all backed by the power of the Qualcomm AI Stack.
        </p>

        <h2>PPE Detection</h2>
        <p>
          Below is an object detection model provided by the QAI Hub.
        </p>

        <InferenceFeed webSocketUrl={webSocketUrl}></InferenceFeed>
        </div>
    </div>
  )
}

export default QAIHubPage
