/* -----------------------------------------------------------------------------

  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
  SPDX-License-Identifier: BSD-3-Clause
 
   ----------------------------------------------------------------------------- */


import { BrowserRouter, Routes, Route } from 'react-router-dom';
import HomePage from './pages/Home';
import EdgeImpulsePage from './pages/EdgeImpulse';
import QAIHubPage from './pages/QAIHub';


function App() {
  return (
    <BrowserRouter>
      <Routes> 
        <Route path="/" element={<HomePage />} />
        <Route path="/edge-impulse" element={<EdgeImpulsePage />} />
        <Route path="/qai-hub" element={<QAIHubPage />} />
      </Routes>
    </BrowserRouter>
  );
}

export default App
