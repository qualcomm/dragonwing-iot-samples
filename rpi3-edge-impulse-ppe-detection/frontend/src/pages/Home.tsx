/* -----------------------------------------------------------------------------

  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
  SPDX-License-Identifier: BSD-3-Clause
 
   ----------------------------------------------------------------------------- */

   
import { useState, useEffect } from 'react';
import NavBar from '../components/NavBar';

function HomePage() {
  const [isMobile, setIsMobile] = useState(window.innerWidth < 768);

  useEffect(() => {
    const handleResize = () => {
      setIsMobile(window.innerWidth < 768);
    };

    window.addEventListener('resize', handleResize);

    // Cleanup listener on unmount
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  const imgGridCardWidth = isMobile ? 140 : undefined;
  const imgGridCardGap = isMobile ? "7px" : "50px";
  const imgWidth = isMobile ? 140 : 320;

  return (
    <div>
        <NavBar/>

        <div className="center-box">
        <h1>SafetyVision</h1>
        <p>A proof of concept that showcases the detection of PPE equipment. This proof of concept uses on-device AI capabilties to dectect PPE objects in real-time using both an Edge Impulse model and QAI Hub model.</p>

        <h2>Possible States</h2>
        <p>There are four possible different states this proof of concept can detect:</p>
        <div className="img-grid" style={{gap: imgGridCardGap}}>
          <div style={{width: imgGridCardWidth}}>
            <img src="/no_hat_no_vest.png" width={imgWidth} className="img-grid-img"></img>
            <p>❌ Hard hat <br></br>❌ Safety vest</p>
          </div>
          <div style={{width: imgGridCardWidth}}>
            <img src="/hat_no_vest.png" width={imgWidth} className="img-grid-img"></img>
            <p>✅ Hard hat <br></br>❌ Safety vest</p>
          </div>
          <div style={{width: imgGridCardWidth}}>
            <img src="/no_hat_vest.png" width={imgWidth} className="img-grid-img"></img>
            <p>❌ Hard hat <br></br>✅ Safety vest</p>
          </div>
          <div style={{width: imgGridCardWidth}}>
            <img src="/hat_vest.png" width={imgWidth} className="img-grid-img"></img>
            <p>✅ Hard hat <br></br>✅ Safety vest</p>
          </div>
        </div>

        <h2>Try Out The Models</h2>
        <p>To try out the models, visit the following subpages:</p>
        <div className="model-links">
          <a href="/edge-impulse" className="model-card">
            <h3>Edge Impulse Model</h3>
            <p>Real-time PPE detection using Edge Impulse.</p>
          </a>
          <a href="/qai-hub" className="model-card">
            <h3>QAI Hub Model</h3>
            <p>On-device AI with Qualcomm's QAI Hub.</p>
          </a>
        </div>
        </div>
    </div>
  )
}

export default HomePage
