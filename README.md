# A-Maze-ing

2025-02 유레카 공학설계입문

Loop가 있는 미로 해결 프로젝트 

## 1. Theoritial background
### 1.1. Maze 
A maze is an nndirected graph $ G = (V, E)$ where start node $s \in V$, exit node $t \in V$

### 1.2. Exploration path
A path from $s$ to $t$ is $P = (v_0, v_1, ... v_k)$ where $s = v_0$, $t = v_k$, $(v_i, v_{i+1}) \in E $ s.t. $\forall i \in [0, k]$

### 1.3. Loop(cycle)
A cycle is a sequence $(w_0, w_1, ... , w_m, w_0), \;\; (m\geq 2)$

### 1.4. Tremaux edge making
Each edge traking $f: E \to \{0, 1, 2 \}$ count visits