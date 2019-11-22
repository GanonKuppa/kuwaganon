
#pragma once

void initWebAppConnection();
void finalizeWebAppConnection();
void sendRobotPos(float x, float y, float ang, float v);
void sendTargetPos(float x, float y, float ang);
void sendMazeWall(uint32_t* walls_vertical, uint32_t* walls_horizontal);
void sendNeedle(float x, float y);
void sendReload();
void sendNumberSqure(float num, float x, float y);
