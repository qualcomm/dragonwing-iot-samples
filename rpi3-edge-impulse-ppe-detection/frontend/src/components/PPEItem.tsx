/* -----------------------------------------------------------------------------

  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
  SPDX-License-Identifier: BSD-3-Clause
 
   ----------------------------------------------------------------------------- */
   

export interface PPEItemInfo {
  name: string,
  detected: boolean
  confidence: number
}

export const PPEItem = ({ itemInfo }: { itemInfo: PPEItemInfo }) => {
  return (
    <div>
      <p style={{ marginBottom: "0.5rem" }}>{itemInfo.detected ? "✅" : "❌"} {itemInfo.name} {itemInfo.confidence != -1 ? `(${(itemInfo.confidence * 100).toFixed(2)}% confidence)` : ""}</p>
    </div >
  );
};

export default PPEItem;
