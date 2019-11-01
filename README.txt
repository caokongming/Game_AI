Team members:
Kongming Cao 661850357
Hongyang Lin 661613896

To start the game, use "Field" in the scene directory, and press each number key to start each part. 

1. 
separation weight = 10;
cohesion weight = 1;
alignment weight = 1;

2. Each agent detects closest obstables and avoid using collision prediction / avoidance.
follow path weight = 5;
separation weight = 10;
cohesion weight = 1;
alignment weight = 1;

3. I only used three lines. Three parallel lines are sufficient in most cases and can handle tunnel really well. More raycasts make agent less willing to go into the tunnel.

Note: Use "E" to start action. "S" is occupied by player movement.
"R": restart.
Part 2: "C" to use collision avoidance, "P" to use collision prediction.
