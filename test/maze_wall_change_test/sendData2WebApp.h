
#pragma once

void initWebAppConnection();
void finalizeWebAppConnection();
void sendRobotPos(float x, float y, float ang);
void sendTargetPos(float x, float y, float ang);
void sendMazeWall(uint32_t* walls_vertical, uint32_t* walls_horizontal);


