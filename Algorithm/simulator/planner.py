from simulator.types import Command, Obstacle

OBSTACLES: list[Obstacle] = [
    Obstacle(x=50, y=50, face='N'),
    Obstacle(x=100, y=30, face='E'),
    Obstacle(x=150, y=80, face='S'),
    Obstacle(x=80, y=130, face='W'),
    Obstacle(x=130, y=160, face='N'),
]


def get_commands(obstacles: list[Obstacle]) -> list[Command]:
    # Stage 1: hardcoded sequence to exercise the animation loop.
    # Stage 2 replaces this body with Dubins path + Hamiltonian ordering.
    return [
        Command('FW', 50),
        Command('TR', 90),
        Command('FW', 60),
        Command('TL', 45),
        Command('FW', 40),
        Command('TR', 90),
        Command('BW', 20),
        Command('TL', 90),
        Command('FW', 80),
    ]
