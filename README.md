# DQN-based contention-window control for Wi-Fi 6 in ns-3

This repository contains the reinforcement-learning bonus experiment from [Computer-Network CA2](https://github.com/maleki-shayan/Computer-Network/tree/main/CA2). CA2 compared Wi-Fi 5 and Wi-Fi 6 performance in ns-3; this extracted project focuses on one additional question: **can a DQN agent adapt the Wi-Fi 6 stations' best-effort contention window from live simulation feedback?** It is an experiment, not a complete Wi-Fi controller or a standalone ns-3 distribution.

## What is simulated

[`scratch/wifi6.cc`](scratch/wifi6.cc) creates one 802.11ax access point and 40 stations. Stations send UDP traffic to the AP over a 10-second simulation. The scenario uses uplink OFDMA and BSRP, and reports sent/received/lost packets, throughput, delay, Jain's fairness index, and per-station received SINR at the end.

Every **0.1 seconds of simulation time**, the simulator sends the recent transmitted-packet count, received-packet count, and current contention window to the Python agent over UDP at `127.0.0.1:9999`:

```text
TX=<count>_RX=<count>_CW=<value>
```

[`dqn_agent.py`](dqn_agent.py) replies with one of three actions: `0` to reduce the contention window, `1` to keep it, or `2` to increase it. The simulator applies the resulting window to the stations' best-effort access configuration. The agent's reward is the received-packet count divided by 100 for each decision interval. Its state therefore has **three values** (`TX`, `RX`, `CW`); it does not directly optimize the other reported metrics.

The agent uses a small PyTorch deep Q-network, epsilon-greedy exploration while training, experience replay, and a target network. Evaluation mode loads the saved weights and chooses actions greedily without training. The simulator waits for each reply before continuing, so the Python agent must be running first.

## Repository contents

| Path | Purpose |
| --- | --- |
| `scratch/wifi6.cc` | ns-3 Wi-Fi 6 scenario, UDP bridge, contention-window updates, and final statistics. |
| `dqn_agent.py` | DQN training/evaluation process and TensorBoard logging. |
| `uora_dqn.pt` | Included trained weights used by `--eval`. |
| `runs/` | Existing TensorBoard event logs from training/evaluation. |
| `scratch/CMakeLists.txt` | ns-3 scratch build configuration. |

## Requirements and setup

- **ns-3.38** on Linux or WSL, with its normal build prerequisites. This repository only contains selected project files; it does **not** include the full ns-3 source tree or `./ns3` launcher.
- Python 3 with `torch`, `numpy`, and `tensorboard` installed. A GPU is optional for the small DQN; ns-3 runs on the CPU.
- Both processes must run in the **same Linux/WSL environment** so they share `127.0.0.1`. The C++ UDP bridge uses POSIX sockets and is not set up as a native Windows build.

Copy `scratch/wifi6.cc` into the `scratch/` directory of your ns-3.38 installation. Keep `dqn_agent.py` and `uora_dqn.pt` together in this repository directory. From the ns-3 root, build the scenario:

```bash
./ns3 build
```

Install the agent dependencies in a Python environment of your choice, for example:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy torch tensorboard
```

Run the following commands in **two terminals**. Start the agent first and leave it listening; then start the simulator:

```bash
# Terminal 1: from this repository's directory, train a new agent
python3 dqn_agent.py

# Or evaluate the included checkpoint without training
python3 dqn_agent.py --eval
```

```bash
# Terminal 2: from the ns-3.38 root
./ns3 run scratch/wifi6
```

For a new training run, the agent saves `uora_dqn.pt` after every **500 decisions**; one 10-second simulation produces roughly 100 decisions. Stopping training with Ctrl+C instead saves `uora_dqn_interrupted.pt`. To evaluate newly trained weights, ensure they are saved under `uora_dqn.pt` (and preserve the included checkpoint first if you want to keep it). The agent can stay running while you launch additional simulation runs.

View the recorded metrics from this repository's directory with:

```bash
tensorboard --logdir runs
```

Training writes to `runs/uora_training/`; evaluation writes to `runs/uora_evaluation/`. The simulator also prints its network statistics directly to the terminal. The provided checkpoint and logs are artifacts of prior runs, not proof that DQN outperforms a fixed contention window; a controlled baseline comparison is needed for that claim.

## Relationship to the full CA2

The [original CA2 project](https://github.com/maleki-shayan/Computer-Network/tree/main/CA2) also contains the Wi-Fi 5/Wi-Fi 6 comparison phases and assignment material. Those phases are **not** included here. Use that repository for the broader coursework context; use this one for the extracted DQN/Wi-Fi 6 experiment.
