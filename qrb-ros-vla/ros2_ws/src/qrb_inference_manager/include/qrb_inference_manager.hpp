// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_INFERENCE_MANAGER_QRB_INFERENCE_MANAGER_HPP_
#define QRB_INFERENCE_MANAGER_QRB_INFERENCE_MANAGER_HPP_

#include "qrb_inference.hpp"

namespace qrb::inference_mgr
{

class QrbInferenceManager
{
public:
  // device_id picks the HTP hardware device (0 or 1 on QCS9075). Ignored for
  // .tflite models, which go through the TFLite delegate path.
  QrbInferenceManager(const std::string & model_path,
      const std::string & backend_option = "",
      uint32_t device_id = 0);
  ~QrbInferenceManager() = default;
  bool inference_execute(const std::vector<uint8_t> & input_tensor_data);
  bool inference_execute_dmabuf(int dmabuf_fd, uint32_t dmabuf_size, uint64_t dmabuf_offset = 0);
  std::vector<OutputTensor> get_output_tensors();

private:
  std::unique_ptr<QrbInference> qrb_inference_{ nullptr };
};  // class QrbInferenceManager

}  // namespace qrb::inference_mgr

#endif
