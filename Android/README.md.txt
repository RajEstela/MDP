# MDP Remote Controller

Android Kotlin Jetpack Compose starter for an MDP robot remote controller.

## Packages

- `bluetooth`: paired-device discovery, Bluetooth SPP/RFCOMM connection, text send/receive.
- `ui`: Compose app shell, controller screen, ViewModel state.
- `map`: 20 x 20 arena grid rendering and obstacle dragging.
- `robot`: movement actions and high-level robot command types.
- `commandparser`: wire command formatter/parser.
- `model`: arena, robot, obstacle, direction, and Bluetooth device models.

## Command Examples

The formatter emits commands such as:

```text
ADD,B1,(10,6)
SUB,B1
FACE,B2,N
TARGET,B2,11,N
```

Raw commands can also be typed and sent directly from the Compose UI.

The Arena tab can send the complete robot and obstacle configuration as a
newline-terminated JSON snapshot. The Raspberry Pi caches this snapshot and
relays it to the laptop algorithm over TCP port `5001`.
