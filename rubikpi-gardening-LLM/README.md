# **Gardening Advisor LLM on Rubik Pi**
## **Fine-tuning Qwen with LoRA on a Gardening dataset for running on Rubik Pi using Llama.cpp**
Fine-tuning involves taking a pre-trained language model (which has already learned a vast amount of general knowledge and language patterns from a diverse dataset) and further training it on a smaller, task-specific, or domain-specific dataset to improve performance and coherence in responses.

This project demonstrates how to fine-tune an LLM ([Qwen2.5-3B-Instruct](https://huggingface.co/Qwen/Qwen2.5-3B-Instruct)) using LoRA (Low-Rank Adaptation) on a [gardening specific dataset](https://huggingface.co/datasets/mlfoundations-dev/stackexchange_gardening).  LoRA (Low-Rank Adaptation) is a parameter-efficient fine-tuning technique that introduces small trainable matrices into certain layers, allowing most of the original model parameters to remain unchanged. This approach drastically reduces memory and computing requirements, making LoRA much faster and more efficient than full finetuning, especially for large language models.

The [fine-tuned model](https://huggingface.co/sanjana040/qwen3B-gardening-finetuned) is then uploaded to huggingface and downloaded to Rubik Pi for inference. Llama.cpp is a high-performance inference engine written in C/C++, tailored for running Llama and compatible models in the GGUF format. Models running under llama.cpp are optimized for GPU execution. 


# **Getting Started** 

# **Requirements** 
•	Python (version 3.10+)

•	Rubik Pi w/Ubuntu 24.04 - Follow Rubik Pi set up [instructions](https://www.thundercomm.com/rubik-pi-3/en/docs/about-rubikpi/)

•	Install llama.cpp and its dependencies from this [link](https://www.thundercomm.com/rubik-pi-3/en/docs/rubik-pi-3-user-manual/1.0.0-u/Application%20Development%20and%20Execution%20Guide/Framework-Driven%20AI%20Sample%20Execution/llama_cpp)

•	Install required libraries in colab notebook -  transformers, datasets, peft, accelerate & huggingface_hub 
```bash
(!pip install transformers datasets peft accelerate huggingface_hub)
```
•	Install dependencies on Rubik Pi – huggingface_hub, transformers, mistral_common, torch & sentencepiece 



# **Run the py notebook -**
Execute all cells in the notebook after installing the above mentioned dependencies: 

•	Fine-tune Qwen2.5-3B-Instruct using LoRA.

•	Merge LoRA weights into the base model.

•	Upload the merged model to Hugging Face Hub. (Make sure the model repo you create on Hugging Face is public so the model can be downloaded later)

# **Rubik Pi Deployment**
## **Download Model on Rubik Pi –** 
Link for hugging face model [sanjana040/qwen3B-gardening-finetuned · Hugging Face](https://huggingface.co/sanjana040/qwen3B-gardening-finetuned)

SSH into your Rubik pi device and navigate to the llama.cpp folder -
```bash
cd dev/llm/llama.cpp
```
Create a virtual environment in your project directory - 
```bash
python -m venv .env
```
Activate the virtual environment -
```bash
source .env/bin/activate
```


***
 ```bash 
# Install Hugging Face CLI
pip install huggingface_hub

# Login with your HF token
hf auth login

# Download the merged model from Hugging Face
hf download sanjana040/qwen3B-gardening-finetuned --local-dir qwen3B-gardening
```





## **Quantize the model** 

To run GPU-accelerated models you'll want pure 4-bit quantized (Q4_0) models in GGUF format.

First convert the model to GGUF format and then quantize to pure 4 bit.
```bash
python3 convert_hf_to_gguf.py qwen3B-gardening --outfile qwen3B-converted.gguf
```
Next, Quantize the downloaded model using llama-quantize. 
```bash
llama-quantize --pure qwen3B-converted.gguf qwen3B-quantized-q4.gguf Q4_0
```
## **Run the LLM using llama-cli**
Llama CLI is a command-line tool for running, managing, and interacting with Large Language Models (LLMs). 

Average Model performance – 6-7 tokens per second

**To start generating text with the quantized local model:**
```bash
llama-cli -m ./qwen3B-quantized-q4.gguf -no-cnv --no-warmup -b 128 -c 2048 -s 11 -n 250 -p "how to grow an avocado plant? " -fa off
```
•	-m specifies path to the model file in GGUF format. 

•	-p sets the prompt.

•	-n controls the number of tokens to generate.

•	-no-cnv  Disables conversation mode. Normally, llama-cli can maintain multi-turn chat context; this flag forces it to treat the prompt as a single-shot query.

•	--no-warmup Skips the initial warm-up phase (used for benchmarking or cache priming). This reduces startup time.

•	-b 128 Batch size for token processing. Larger batch sizes can improve throughput but require more RAM.

•	-c 2048 Context length (number of tokens the model can consider). Here, 2048 tokens means the model can handle fairly long prompts and responses.

•	-s 11 Random seed for reproducibility. Same seed → same output for identical settings.

•	-fa off Disables flash attention optimization 
 


## **Serve the model using llama-server**
We can make use of the llama-server to start a web server with a chat interface.

Average Model Performance – 5 tokens per second 

1)	Find the IP address of your development board -

hostname -I

2)	Start the server – 
```bash
llama-server -m ./qwen3B-quantized-q4.gguf --no-warmup -b 128 -c 2048 -s 11 -n 500 --host 0.0.0.0 --port 9876 --no-prefill-assistant
```
3)	Open a web browser and navigate to http://`<your device IP address>`:9876


