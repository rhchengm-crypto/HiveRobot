# Continue This Project On Mac

This repo is the shared handoff point between the Windows workspace, a Mac development machine, and the robot/Orin runtime.

## 1. Install Git On Mac

Open Terminal and run:

```bash
git --version
```

If macOS asks to install command line developer tools, accept it. You can also install Git through Homebrew:

```bash
brew install git
```

## 2. Clone The Project

Choose a local folder, then clone:

```bash
mkdir -p ~/Projects
cd ~/Projects
git clone https://github.com/rhchengm-crypto/HiveRobot.git
cd HiveRobot
```

After cloning, confirm the repo is clean:

```bash
git status
```

Expected result:

```text
On branch main
Your branch is up to date with 'origin/main'.
nothing to commit, working tree clean
```

## 3. Open In Codex Or VS Code

From the repo folder:

```bash
code .
```

If using Codex Desktop on Mac, open the local folder:

```text
~/Projects/HiveRobot
```

The ChatGPT/Codex account can carry conversation context, but the actual project files come from this GitHub repo.

## 4. Main Project Areas

- `roboparty_left_arm_plan/`: left-arm control, data collection, replay, and behavior-cloning scripts.
- `Nuwa-HP60C-Depth-Camera-main/`: HP60C depth-camera documentation and tutorials.
- `roboparty_rpo_hardware/`: robot hardware CAD, PCB, assembly docs, URDF, and manufacturing files.
- `*.xlsx`: project notes, procurement, sensor memo, and motor ID files.

## 5. Python Setup For Local Script Work

On Mac, use a local virtual environment for editing and non-hardware utilities:

```bash
cd ~/Projects/HiveRobot
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy opencv-python
```

Some hardware scripts import `DM_CAN.py` from `DM_Control_Python`. Those scripts are meant to run from the robot/Orin software folder, not directly from this repo root on Mac unless that dependency is available.

## 6. Robot/Orin Runtime Layout

For real motor tests, place or copy the left-arm folder under:

```bash
~/hive_robot/DM_Control_Python/roboparty_left_arm_plan
```

Run from:

```bash
cd ~/hive_robot/DM_Control_Python
```

Example commands:

```bash
sudo python3 roboparty_left_arm_plan/collect_scripted_demo.py --out data/left_arm_demo.csv
sudo python3 roboparty_left_arm_plan/collect_multimodal_demo.py --out-dir data/left_arm_mm_demo
sudo python3 roboparty_left_arm_plan/replay_demo.py --csv data/left_arm_demo.csv --speed-scale 0.4
python3 roboparty_left_arm_plan/train_bc_ridge.py --csv data/left_arm_demo.csv --out data/left_arm_bc_model.json
```

## 7. Daily Sync Workflow

Before editing on Mac:

```bash
git pull
```

After editing:

```bash
git status
git add .
git commit -m "Describe the change"
git push
```

Back on Windows:

```powershell
git pull
```

Only edit from one machine at a time unless you are comfortable resolving merge conflicts.

## 8. Large File Note

The repo contains CAD and documentation assets. GitHub accepted the current push, but files larger than 50 MB trigger warnings. Avoid adding files over 100 MB directly to Git. Use Git LFS or keep very large archives outside the repo.
