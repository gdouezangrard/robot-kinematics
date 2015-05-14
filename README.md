# Virtual Quadruped Robot

- Inverse kinematics.
- Cubic spline interpolation.
- Realtime control from terminal.

![Demo](assets/output.gif)

## Requirements

- V-REP (with legacy remote API)
- cmake
- ncurses

## Build

1. Download [V-REP PRO EDU](https://www.coppeliarobotics.com/previousVersions)
   from Coppelia Robotics, and extract it.
2. Change directory to `build`.
3. Run `cmake -D VREP_ROOT=PATH ..`, where `PATH` points to V-REP root
   directory.
4. Run `make`.

## Run

1. Edit V-REP remote API settings in `VREP_ROOT/remoteApiConnections.txt`:

   ```
   portIndex1_port = 4242
   portIndex1_debug = true
   portIndex1_syncSimTrigger = true
   ```

2. Start V-REP with `./vrep.sh`.
3. Open `assets/MainScene.ttt` and start the simulation.
4. Start the client with `./client 127.0.0.1 4242`.

## Scene

- Lua [remote API](assets/api.lua) functions attached to a dummy node labeled
  `remoteApi`.
- [metabot](https://github.com/Rhoban/Metabot/tree/v1) model from
  `assets/metabot` imported with `v_repExtUrdf`.

## Acknowledgement

- [RhobanProject/Vrep](https://github.com/RhobanProject/Vrep): the add-on for
  the legacy remote API has been replaced with a Lua child script called with
  `simxCallScriptFunction`. The library has been merged into this project with
  a single compilation step.
