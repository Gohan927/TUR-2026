#include "Maze.h"

void Maze::begin() {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            grid[x][y].wallNorth = false;
            grid[x][y].wallEast  = false;
            grid[x][y].wallSouth = false;
            grid[x][y].wallWest  = false;
            grid[x][y].floodValue = FLOOD_INFINITY;
            grid[x][y].visited = false;
        }
    }

    // Paredes externas do labirinto (bordas) sao sempre conhecidas de
    // antemao -- todo labirinto de competicao e fechado nas bordas.
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        grid[x][0].wallSouth = true;
        grid[x][MAZE_SIZE - 1].wallNorth = true;
    }
    for (uint8_t y = 0; y < MAZE_SIZE; y++) {
        grid[0][y].wallWest = true;
        grid[MAZE_SIZE - 1][y].wallEast = true;
    }

    // Posicao inicial padrao: canto (0,0), apontando para NORTH.
    // Ajustar conforme a regra especifica da competicao, se diferente.
    robotX = 0;
    robotY = 0;
    robotDirection = NORTH;

    recalculateFloodFill();
}

bool Maze::isValidCell(int x, int y) const {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

Cell& Maze::cellAt(uint8_t x, uint8_t y) {
    return grid[x][y];
}

bool Maze::hasWallBetween(uint8_t x, uint8_t y, Direction dir) const {
    switch (dir) {
        case NORTH: return grid[x][y].wallNorth;
        case EAST:  return grid[x][y].wallEast;
        case SOUTH: return grid[x][y].wallSouth;
        case WEST:  return grid[x][y].wallWest;
    }
    return true; // seguranca: direcao invalida tratada como parede
}

Direction Maze::relativeLeftOf(Direction facing) const {
    // Rotacao anti-horaria (esquerda) no ciclo N->W->S->E->N
    return (Direction)((facing + 3) % 4);
}

Direction Maze::relativeRightOf(Direction facing) const {
    // Rotacao horaria (direita) no ciclo N->E->S->W->N
    return (Direction)((facing + 1) % 4);
}

void Maze::updateWallsAtCurrentCell(bool wallLeft, bool wallFront, bool wallRight) {
    Direction absLeft  = relativeLeftOf(robotDirection);
    Direction absFront = robotDirection;
    Direction absRight = relativeRightOf(robotDirection);

    Cell &current = grid[robotX][robotY];

    // Regra: so ADICIONA parede (nunca remove uma ja conhecida).
    // Isso protege o mapa contra uma leitura pontual invalida/ruidosa
    // que "esqueceu" de detectar uma parede real.
    auto setWallIfDetected = [&](Direction dir, bool detected) {
        if (!detected) return; // nao apaga paredes existentes
        switch (dir) {
            case NORTH: current.wallNorth = true; break;
            case EAST:  current.wallEast  = true; break;
            case SOUTH: current.wallSouth = true; break;
            case WEST:  current.wallWest  = true; break;
        }
    };

    setWallIfDetected(absLeft, wallLeft);
    setWallIfDetected(absFront, wallFront);
    setWallIfDetected(absRight, wallRight);

    // Espelha a parede na celula vizinha tambem (uma parede e compartilhada
    // entre duas celulas adjacentes -- manter consistencia do mapa).
    auto mirrorWall = [&](uint8_t x, uint8_t y, Direction dir) {
        int nx = x, ny = y;
        Direction oppositeDir = NORTH; // valor padrao de seguranca (nunca deveria ser usado)
        switch (dir) {
            case NORTH: ny = y + 1; oppositeDir = SOUTH; break;
            case EAST:  nx = x + 1; oppositeDir = WEST;  break;
            case SOUTH: ny = y - 1; oppositeDir = NORTH; break;
            case WEST:  nx = x - 1; oppositeDir = EAST;  break;
        }
        if (!isValidCell(nx, ny)) return;
        switch (oppositeDir) {
            case NORTH: grid[nx][ny].wallNorth = true; break;
            case EAST:  grid[nx][ny].wallEast  = true; break;
            case SOUTH: grid[nx][ny].wallSouth = true; break;
            case WEST:  grid[nx][ny].wallWest  = true; break;
        }
    };

    if (wallLeft)  mirrorWall(robotX, robotY, absLeft);
    if (wallFront) mirrorWall(robotX, robotY, absFront);
    if (wallRight) mirrorWall(robotX, robotY, absRight);

    current.visited = true;
}

// Flood Fill classico via BFS (fila implementada com array simples,
// suficiente para MAZE_SIZE tipico de competicao).
void Maze::recalculateFloodFill() {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            grid[x][y].floodValue = FLOOD_INFINITY;
        }
    }

    struct QueueItem { uint8_t x; uint8_t y; };
    static QueueItem queue[MAZE_SIZE * MAZE_SIZE];
    uint16_t head = 0, tail = 0;

    grid[GOAL_X][GOAL_Y].floodValue = 0;
    queue[tail++] = { (uint8_t)GOAL_X, (uint8_t)GOAL_Y };

    while (head < tail) {
        QueueItem cur = queue[head++];
        uint8_t curVal = grid[cur.x][cur.y].floodValue;

        // Tenta propagar para as 4 direcoes, respeitando paredes.
        struct { int dx; int dy; Direction dir; } neighbors[4] = {
            {0, 1, NORTH}, {1, 0, EAST}, {0, -1, SOUTH}, {-1, 0, WEST}
        };

        for (uint8_t i = 0; i < 4; i++) {
            int nx = cur.x + neighbors[i].dx;
            int ny = cur.y + neighbors[i].dy;
            if (!isValidCell(nx, ny)) continue;
            if (hasWallBetween(cur.x, cur.y, neighbors[i].dir)) continue;

            if (grid[nx][ny].floodValue > curVal + 1) {
                grid[nx][ny].floodValue = curVal + 1;
                queue[tail++] = { (uint8_t)nx, (uint8_t)ny };
            }
        }
    }
}

NavAction Maze::decideNextAction() {
    if (isAtGoal()) {
        return ACTION_GOAL_REACHED;
    }

    Direction absFront = robotDirection;
    Direction absLeft  = relativeLeftOf(robotDirection);
    Direction absRight = relativeRightOf(robotDirection);
    Direction absBack  = (Direction)((robotDirection + 2) % 4);

    uint8_t currentFlood = grid[robotX][robotY].floodValue;

    // Calcula coordenadas vizinhas em cada direcao absoluta.
    auto neighborCoord = [&](Direction dir, int &nx, int &ny) {
        nx = robotX; ny = robotY;
        switch (dir) {
            case NORTH: ny += 1; break;
            case EAST:  nx += 1; break;
            case SOUTH: ny -= 1; break;
            case WEST:  nx -= 1; break;
        }
    };

    // Avalia cada direcao relativa possivel (sem parede), coletando o
    // floodValue do vizinho correspondente.
    uint8_t bestFlood = FLOOD_INFINITY;
    NavAction bestAction = ACTION_NONE;

    struct Candidate { Direction absDir; NavAction action; };
    Candidate candidates[4] = {
        { absFront, ACTION_FORWARD },
        { absLeft,  ACTION_TURN_LEFT },
        { absRight, ACTION_TURN_RIGHT },
        { absBack,  ACTION_TURN_BACK }
    };

    for (uint8_t i = 0; i < 4; i++) {
        if (hasWallBetween(robotX, robotY, candidates[i].absDir)) continue;

        int nx, ny;
        neighborCoord(candidates[i].absDir, nx, ny);
        if (!isValidCell(nx, ny)) continue;

        uint8_t neighborFlood = grid[nx][ny].floodValue;

        // Preferencia: menor floodValue vence. Em empate, preferimos
        // FRENTE > ESQUERDA/DIREITA > TRAS -- isso minimiza curvas
        // desnecessarias (Confiabilidade e Controle antes de Velocidade:
        // menos curvas = menos chance de erro acumulado de orientacao).
        bool better = (neighborFlood < bestFlood);
        bool tieButBetterAction = (neighborFlood == bestFlood &&
                                    candidates[i].action == ACTION_FORWARD &&
                                    bestAction != ACTION_FORWARD);

        if (better || tieButBetterAction) {
            bestFlood = neighborFlood;
            bestAction = candidates[i].action;
        }
    }

    // Nenhuma direcao valida encontrada -- situacao anormal (celula
    // isolada por paredes de todos os lados, ou mapa inconsistente).
    // A camada de seguranca (Safety.cpp / RobotStateMachine) deve tratar
    // ACTION_NONE como um estado de erro, nunca avancar as cegas.
    if (bestAction == ACTION_NONE) {
        return ACTION_NONE;
    }

    // Nao faz sentido avancar para uma celula com floodValue MAIOR que a
    // atual (indicaria estar se afastando do objetivo) a menos que seja
    // a unica opcao (situacao ja tratada acima, pois so comparamos entre
    // vizinhos SEM parede -- se o unico vizinho livre tiver flood maior,
    // ainda assim seguimos, pois e a unica saida possível).
    (void)currentFlood;

    return bestAction;
}

void Maze::advancePosition() {
    // Protecao contra underflow/overflow: robotX/robotY sao uint8_t, entao
    // decrementar quando ja em 0 (ou incrementar no limite MAZE_SIZE-1)
    // faria "dar a volta" silenciosamente (0 -> 255), corrompendo a
    // posicao logica sem nenhum aviso. Isso so deveria ser fisicamente
    // possivel se o mapa de paredes estiver inconsistente (permitindo
    // avancar por uma borda que deveria ter parede) -- entao tratamos
    // como erro de estado, nunca como movimento silencioso.
    switch (robotDirection) {
        case NORTH:
            if (robotY + 1 < MAZE_SIZE) robotY += 1;
            break;
        case EAST:
            if (robotX + 1 < MAZE_SIZE) robotX += 1;
            break;
        case SOUTH:
            if (robotY > 0) robotY -= 1;
            break;
        case WEST:
            if (robotX > 0) robotX -= 1;
            break;
    }
}

void Maze::turnLogicalDirection(NavAction turnAction) {
    switch (turnAction) {
        case ACTION_TURN_LEFT:
            robotDirection = relativeLeftOf(robotDirection);
            break;
        case ACTION_TURN_RIGHT:
            robotDirection = relativeRightOf(robotDirection);
            break;
        case ACTION_TURN_BACK:
            robotDirection = (Direction)((robotDirection + 2) % 4);
            break;
        default:
            break; // ACTION_FORWARD nao muda orientacao
    }
}

bool Maze::isAtGoal() const {
    return (robotX == GOAL_X && robotY == GOAL_Y);
}
