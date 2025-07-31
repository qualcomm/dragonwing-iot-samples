/* -----------------------------------------------------------------------------

  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
  SPDX-License-Identifier: BSD-3-Clause
 
   ----------------------------------------------------------------------------- */
   

import React, { useState } from 'react';
import './NavBar.css';

const NavBar: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <nav className="navbar">
      <div className="navbar-brand"><a href="/" style={{textDecoration: "none", color: "inherit"}}>SafetyVision</a></div>

      <button className="navbar-toggle" onClick={() => setIsOpen(!isOpen)}>
        ☰
      </button>

      <div className={`navbar-links ${isOpen ? 'open' : ''}`}>
        <a href="/">Home</a>
        <a href="/edge-impulse">Edge Impulse Model</a>
        <a href="/qai-hub">QAI Hub Model</a>
      </div>
    </nav>
  );
};

export default NavBar;
