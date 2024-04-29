# Installation manual

> **Note**
> Ubuntu 20.04 is highly recommended, since the initial version of python3 is **python 3.8** (which is necessary for this package).  

- [Installation manual](#installation-manual)
  - [Install Ros Neotic](#install-ros-neotic)
  - [Install CuriGPT](#install-curiGPT)
    - [Download the package](#download-the-package)
    - [Install requirements](#install-requirements)
  - [Configure APIs](#configure-apis)
    - [Qwen API](#qwen-api)
    - [OpenAI API](#openai-api)


## Install Ros Neotic

Please follow the [official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu) for Ros Neotic, and also [configure your ros environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).


## Install CuriGPT

### Download the package

```
git clone https://github.com/RIP4KOBE/CuriGPT.git
```

### Install requirements

```
cd ~/CuriGPT
python3.8 -m venv curigpt
source curigpt/bin/activate
pip install -r requirements.txt
```

## Configure APIs
### Qwen API

1. Copy your Qwen API from this [website](https://dashscope.aliyun.com/)
2. Export it to environmental variables:
```
export DASHSCOPE_API_KEY=sk-bfe12097afed4249a73cbafb4fec3e1c
```
### OpenAI API

1. Copy your OpenAI API from this [website](https://platform.openai.com/account/api-keys)
2. Paste it to `config/config.json`.