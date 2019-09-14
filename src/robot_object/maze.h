#pragma once

#include "myUtil.h"
#include "wallsensor.h"
#include "stdint.h"
#include <map> // pair
#include <queue>
#include "communication.h"
#include "dataFlash.h"

namespace umouse{

enum direction_e {
    E = 0, NE, N, NW, W, SW, S, SE
};

class Wall {

public:
    uint8_t E :1; //壁情報
    uint8_t N :1;
    uint8_t W :1;
    uint8_t S :1;
    uint8_t EF :1;//壁を読んだかのフラグ
    uint8_t NF :1;
    uint8_t WF :1;
    uint8_t SF :1;

    void setByUint8(uint8_t wall) {
        E = ((wall >> 0) & 0x01);
        N = ((wall >> 1) & 0x01);
        W = ((wall >> 2) & 0x01);
        S = ((wall >> 3) & 0x01);

        EF = ((wall >> 4) & 0x01);
        NF = ((wall >> 5) & 0x01);
        WF = ((wall >> 6) & 0x01);
        SF = ((wall >> 7) & 0x01);
    }

    Wall(){
        setByUint8(0);
    }

    Wall(direction_e dir, bool l, bool a, bool r){
        WF = 1;
        SF = 1;
        EF = 1;
        NF = 1;

        switch(dir) {

        case direction_e::E:  //まうすは東向き
            //W = b;
            S = r;
            E = a;
            N = l;
            break;
        case direction_e::N://まうすは北向き
            //S = b;
            E = r;
            N = a;
            W = l;
            break;
        case direction_e::W://まうすは西向き
            //E = b;
            N = r;
            W = a;
            S = l;
            break;
        case direction_e::S://まうすは南向き
            //N = b;
            W = r;
            S = a;
            E = l;
            break;
        }
    }
};

class Maze {

public:
    uint32_t walls_vertical[31];
    uint32_t walls_horizontal[31];
    uint32_t reached[32];
    uint16_t p_map[32][32];

    bool isReached(uint16_t x, uint16_t y) {
        bool reach;
        if( 0<=x && x<=31 && 0<=y && y<=31) reach =(reached[x] >> y) & 0x00000001;
        else reach = 1;
        return reach;
    };

    void writeReached(uint16_t x, uint16_t y, bool reached_) {
        if( 0<=x && x<=31 && 0<=y && y<=31) {
            if(reached_ == true) reached[x] |= (0x01 << y);
            else reached[x] &= reached[x] &= ~(0x01 << y);
        }
    };

    Wall readWall(uint16_t x, uint16_t y) {
        Wall wall;
        //壁情報の配列番号に変換
        int8_t v_left = x-1;
        int8_t v_right = x;
        int8_t h_up = y;
        int8_t h_down = y-1;
        //壁番号が範囲外の場合は外周の壁
        wall.E = v_right != 31 ? ((walls_vertical[v_right] >>y )& 1) : 1;
        wall.N = h_up != 31 ? ((walls_horizontal[h_up] >>x )& 1) : 1;
        wall.W = v_left != -1 ? ((walls_vertical[v_left] >>y )& 1) : 1;
        wall.S = h_down != -1 ? ((walls_horizontal[h_down]>>x )& 1) : 1;
        wall.EF = isReached(x, y) || isReached(x+1, y );
        wall.NF = isReached(x, y) || isReached(x , y+1);
        wall.WF = isReached(x, y) || isReached(x-1, y );
        wall.SF = isReached(x, y) || isReached(x , y-1);

        return wall;
    };

    bool existRWall(uint16_t x, uint16_t y, direction_e dir){
        Wall wall = readWall(x, y);    
        if(dir == direction_e::E) return wall.S && wall.SF;
        else if(dir == direction_e::N) return wall.E && wall.EF;
        else if(dir == direction_e::W) return wall.N && wall.NF;
        else if(dir == direction_e::S) return wall.W && wall.WF;
        else return false;
    }

    bool existLWall(uint16_t x, uint16_t y, direction_e dir){
        Wall wall = readWall(x, y);
        if(dir == direction_e::E) return wall.N && wall.NF;
        else if(dir == direction_e::N) return wall.W && wall.WF;
        else if(dir == direction_e::W) return wall.S && wall.SF;
        else if(dir == direction_e::S) return wall.E && wall.SF;
        else return false;
    }

    void updateStartSectionWall(){
        writeReached(0,0,true);
        Wall wall = Wall(direction_e::N, true, false, true);
        writeWall(0,0, wall);
    }

    void writeWall(uint16_t x, uint16_t y, Wall wall) {
        //壁情報の配列番号に変換
        int8_t v_left = x-1;
        int8_t v_right = x;
        int8_t h_up = y;
        int8_t h_down = y-1;
        //壁情報を書き込み
        if(v_right != 31) {
            if(wall.E == 1)walls_vertical[v_right] |= (1 << y);
            else walls_vertical[v_right] &= (~(1 << y));
        }
        if(h_up != 31) {
            if(wall.N == 1)walls_horizontal[h_up] |= (1 << x);
            else walls_horizontal[h_up] &= (~(1 << x));
        }
        if(v_left != -1) {
            if(wall.W == 1)walls_vertical[v_left] |= (1 << y);
            else walls_vertical[v_left] &= (~(1 << y));
        }
        if(h_down != -1) {
            if(wall.S == 1)walls_horizontal[h_down] |= (1 << x);
            else walls_horizontal[h_down] &= (~(1 << x));
        }
    };

    void initWall() {
        for(int x=0;x<32;x++) {
            for(int y=0;y<32;y++) {
                p_map[x][y] = 0;
                Wall wall;
                wall.setByUint8(0);
                writeWall(x,y,wall);
            }
        }
    };

    void initReached() {
        for(int i=0;i<32;i++) reached[i] = 0;
    }

    void init(){
        initWall();
        initReached();
    }

    void writeWall(uint16_t x, uint16_t y, direction_e dir, bool l, bool a, bool r){
       Wall wall = readWall(x,y);
       switch(dir) {
          case E:  //まうすは東向き
          wall.W = 0;
          wall.S = r;
          wall.E = a;
          wall.N = l;
          break;
          case N://まうすは北向き
          wall.S = 0;
          wall.E = r;
          wall.N = a;
          wall.W = l;
          break;
          case W://まうすは西向き
          wall.E = 0;
          wall.N = r;
          wall.W = a;
          wall.S = l;
          break;
          case S://まうすは南向き
          wall.N = 0;
          wall.W = r;
          wall.S = a;
          wall.E = l;
          break;
       }
       writeWall(x, y, wall);
    }


    void writeWall(uint16_t x, uint16_t y, direction_e dir, WallSensor& ws){
        bool l = ws.isLeft();
        bool a = ws.isAhead();
        bool r = ws.isRight();
        writeWall(x, y, dir, l, a, r);
    }

    // return 1:  探索済みの区画に来て、迷路情報と現在の壁情報が一致
    // return 0:  未探索の区画に現在の壁情報を書き込み
    // return -1: 探索済みの区画に来たが、現在の迷路情報と現在の壁情報が矛盾
    int8_t updateWall(uint16_t x, uint16_t y, direction_e dir, WallSensor& ws) {
        int16_t left_ = ws.left();
        int16_t ahead_l_ = ws.ahead_l();
        int16_t ahead_r_ = ws.ahead_r();
        int16_t right_ = ws.right();


        if(x == 0 && y == 0){
            writeReached(x, y, true);
            Wall wall;
            wall.E = 1;
            wall.N = 0;
            wall.W = 1;
            writeWall(x, y, wall);
            return 0;
        }

        /////////////探索済み区画に来た時//////////////////
        if(isReached(x,y) == true) {
            //printfAsync("   --- 既探索区画に来た\n");

            switch(dir) {
                case E:
                    //printfAsync("   E:%d %d\n",readWall(x,y).E,ws.isAhead() );
                    //printfAsync("   S:%d %d\n",readWall(x,y).S,ws.isRight() );
                    //printfAsync("   N:%d %d\n",readWall(x,y).N,ws.isLeft() );

                    if(readWall(x,y).E != ws.isAhead() ||
                            readWall(x,y).S != ws.isRight() ||
                            readWall(x,y).N != ws.isLeft()
                        ) return -1;
                    break;
                case N:
                    //printfAsync("   N:%d %d\n",readWall(x,y).N,ws.isAhead() );
                    //printfAsync("   E:%d %d\n",readWall(x,y).E,ws.isRight() );
                    //printfAsync("   W:%d %d\n",readWall(x,y).W,ws.isLeft() );

                    if(readWall(x,y).N != ws.isAhead() ||
                            readWall(x,y).E != ws.isRight() ||
                            readWall(x,y).W != ws.isLeft()) return -1;
                break;
                case W:
                    //printfAsync("   W:%d %d\n",readWall(x,y).W,ws.isAhead() );
                    //printfAsync("   N:%d %d\n",readWall(x,y).N,ws.isRight() );
                    //printfAsync("   S:%d %d\n",readWall(x,y).S,ws.isLeft() );

                    if(readWall(x,y).W != ws.isAhead()||
                            readWall(x,y).N != ws.isRight()||
                            readWall(x,y).S != ws.isLeft()) return -1;
                    break;
                case S:
                    //printfAsync("   S:%d %d\n",readWall(x,y).S,ws.isAhead() );
                    //printfAsync("   W:%d %d\n",readWall(x,y).W,ws.isRight() );
                    //printfAsync("   E:%d %d\n",readWall(x,y).E,ws.isLeft() );

                    if(readWall(x,y).S != ws.isAhead() ||
                            readWall(x,y).W != ws.isRight()||
                            readWall(x,y).E != ws.isLeft()) return -1;
                    break;

            }
            return 1;
        }
        ///////////未探索区画に来た時//////////
        else {
            writeReached(x,y,true);
            writeWall(x, y, dir, ws);
            return 0;
        } //end else
    }

    int8_t updateWall(uint16_t x, uint16_t y, direction_e dir, bool l, bool a, bool r) {
         if(x==0 && y==0){
             writeReached(x,y,true);
             Wall wall;
             wall.E = 1;
             wall.N = 0;
             wall.W = 1;
             writeWall(x, y, wall);
             return 0;
         }

         writeReached(x,y,true);
         writeWall(x, y, dir, l, a, r);
         return 1;

     }

    direction_e getMinDirection(uint16_t x, uint16_t y, direction_e dir) {
        uint16_t potential_E = 0xffff;
        uint16_t potential_N = 0xffff;
        uint16_t potential_W = 0xffff;
        uint16_t potential_S = 0xffff;

        Wall wall = readWall(x,y);
        if(wall.E == 0 && wall.EF == 1 && x != 31)potential_E = p_map[x+1][y];
        if(wall.N == 0 && wall.NF == 1 && y != 31)potential_N = p_map[x][y+1];
        if(wall.W == 0 && wall.WF == 1 && x != 0)potential_W = p_map[x-1][y];
        if(wall.S == 0 && wall.SF == 1 && y != 0)potential_S = p_map[x][y-1];
        uint16_t potential_min = MIN4( potential_E, potential_N, potential_W, potential_S );

        //直進有線にしている
        if( (potential_min == potential_E) && (dir == E) ) return E;
        if( (potential_min == potential_N) && (dir == N) ) return N;
        if( (potential_min == potential_W) && (dir == W) ) return W;
        if( (potential_min == potential_S) && (dir == S) ) return S;
        if(potential_min == potential_E) return E;
        if(potential_min == potential_N) return N;
        if(potential_min == potential_W) return W;
        if(potential_min == potential_S) return S;
    };

    uint8_t getUnknownDirection(uint16_t x, uint16_t y, direction_e dir) {
        uint8_t ran_judge_E = 0;
        uint8_t ran_judge_N = 0;
        uint8_t ran_judge_W = 0;
        uint8_t ran_judge_S = 0;

        Wall wall = readWall(x,y);
        if( (x != 31) && (isReached(x+1,y) == false) && (wall.E == 0) ) {
            ran_judge_E = 1;
        }
        if( (y != 31) && (isReached(x,y+1) == false) && (wall.N == 0) ) {
            ran_judge_N = 1;
        }
        if( (x != 0) && (isReached(x-1,y) == false) && (wall.W == 0) ) {
            ran_judge_W = 1;
        }
        if( (y != 0) && (isReached(x,y-1) == false) && (wall.S == 0)) {
            ran_judge_S = 1;
        }

        if( (ran_judge_E == 1) && (dir == E) ) return 0;
        if( (ran_judge_N == 1) && (dir == N) ) return 2;
        if( (ran_judge_W == 1) && (dir == W) ) return 4;
        if( (ran_judge_S == 1) && (dir == S) ) return 6;

        if(ran_judge_E == 1) return 0;
        if(ran_judge_N == 1) return 2;
        if(ran_judge_W == 1) return 4;
        if(ran_judge_S == 1) return 6;

        return 255;
    }

    direction_e getSearchDirection(uint16_t x, uint16_t y, direction_e dir){
        uint8_t min_dir = (uint8_t)getMinDirection(x, y, dir);
        uint8_t unknown_dir = (uint8_t)getUnknownDirection(x, y, dir);
        if(unknown_dir == 255) return (direction_e)min_dir;
        else if(unknown_dir != min_dir) return (direction_e)unknown_dir;
        else return (direction_e)min_dir;
    }

    void makeSearchMap(uint16_t x, uint16_t y) {
        std::queue<std::pair<uint16_t, uint16_t>> que;

        //歩数マップの初期化
        for(uint8_t i=0;i<32;i++) {
            for(uint8_t j=0;j<32;j++) {
                p_map[i][j] = 0xffff;
            }
        }
        p_map[x][y] = 0; //目的地のテンシャルは0

        que.push(std::make_pair(x,y));
        while(que.empty() == false) {
            x = que.front().first;
            y = que.front().second;
            Wall wall = readWall((uint16_t)(que.front().first), (uint16_t)(que.front().second));
            que.pop();
            if( (wall.E == 0) && (x != 31) && (p_map[x+1][y] == 0xffff) ) {
                p_map[x+1][y] = p_map[x][y] + 1;
                que.push(std::make_pair(x+1,y));
            }
            if( (wall.N == 0 ) && (y != 31) && (p_map[x][y+1] == 0xffff) ) {
                p_map[x][y+1] = p_map[x][y] + 1;
                que.push(std::make_pair(x,y+1));
            }
            if( (wall.W == 0) && (x != 0) && (p_map[x-1][y] == 0xffff) ) {
                p_map[x-1][y] = p_map[x][y] + 1;
                que.push(std::make_pair(x-1,y));
            }
            if( (wall.S == 0) && (y != 0) && (p_map[x][y-1] == 0xffff) ) {
                p_map[x][y-1] = p_map[x][y] + 1;
                que.push(std::make_pair(x,y-1));
            }
        }

    };
    //
    void makeSecondarySerchMap(uint16_t x, uint16_t y);
    bool isExistPath(uint16_t x, uint16_t y){
        if(p_map[x][y] == 0xffff) return false;
        else return true;
    }
    void makeFastestMap(uint16_t x, uint16_t y) {
        std::queue<std::pair<uint16_t, uint16_t>> que;

        //歩数マップの初期化
        for(uint8_t i=0;i<32;i++) {
            for(uint8_t j=0;j<32;j++) {
                p_map[i][j] = 0xffff;
            }
        }
        p_map[x][y] = 0; //目的地のテンシャルは0

        que.push(std::make_pair(x,y));
        while(que.empty() == false) {
            x = que.front().first;
            y = que.front().second;
            Wall wall = readWall((uint16_t)(que.front().first), (uint16_t)(que.front().second));
            que.pop();
            if( (wall.E == 0) && (wall.EF == 1) && (x != 31) && (p_map[x+1][y] == 0xffff) ) {
                p_map[x+1][y] = p_map[x][y] + 1;
                que.push(std::make_pair(x+1,y));
            }
            if( (wall.N == 0 ) && (wall.NF == 1 ) && (y != 31) && (p_map[x][y+1] == 0xffff) ) {
                p_map[x][y+1] = p_map[x][y] + 1;
                que.push(std::make_pair(x,y+1));
            }
            if( (wall.W == 0) && (wall.WF == 1) && (x != 0) && (p_map[x-1][y] == 0xffff) ) {
                p_map[x-1][y] = p_map[x][y] + 1;
                que.push(std::make_pair(x-1,y));
            }
            if( (wall.S == 0) && (wall.SF == 1) && (y != 0) && (p_map[x][y-1] == 0xffff) ) {
                p_map[x][y-1] = p_map[x][y] + 1;
                que.push(std::make_pair(x,y-1));
            }
        }

    };


    void watchPotentialMap(void) {
        /////////////////////////////
        for(int j=15;j>=0;j--) {
            for(int i=0;i<16;i++) {
                //////1桁//////////
                if(p_map[i][j]< 10) {
                    if(i==0) {
                        printfAsync("%x  %d    ",j, p_map[i][j]);
                    }
                    else if(i==15) {
                        printfAsync("%d\n",p_map[i][j]);
                    }
                    else {
                        printfAsync("%d    ",p_map[i][j]);
                    }
                }
                /////2桁//////////
                if( (p_map[i][j] > 9) && (p_map[i][j] < 99) ) {
                    if(i==0) {
                        printfAsync("%x  %d   ",j, p_map[i][j]);
                    }
                    else if(i==15) {
                        printfAsync("%d\n",p_map[i][j]);
                    }
                    else {
                        printfAsync("%d   ",p_map[i][j]);
                    }
                }
                /////3桁//////////
                if(p_map[i][j] > 100) {
                    if(i==0) {
                        printfAsync("%x  %d  ",j, p_map[i][j]);
                    }
                    else if(i==15) {
                        printfAsync("%d\n",p_map[i][j]);
                    }
                    else {
                        printfAsync("%d  ",p_map[i][j]);
                    }
                }

            }
        }
        printfAsync("\n   0    1    2    3    4    5    6    7    8    9    a    b    c    d    e    f   \n");
    }


    int8_t calcRotTimes(direction_e dest_dir, direction_e my_dir){
        int8_t rot_times = dest_dir - my_dir;
        if (rot_times == 6) rot_times = -2;
        else if (rot_times == -6) rot_times = 2;
        else if (rot_times == -4) rot_times = 4;
        return rot_times;

    }

    //
    void serializeMazeData(uint8_t* byte_arr){
        uint8_t* p;

        p = reinterpret_cast<uint8_t *>(&walls_vertical[0]);
        for(uint16_t i=0; i<sizeof(walls_vertical); i++) byte_arr[i] = p[i];

        p = reinterpret_cast<uint8_t *>(&walls_horizontal[0]);
        for(uint16_t i=0; i<sizeof(walls_horizontal); i++) byte_arr[i+sizeof(walls_vertical)] = p[i];

        p = reinterpret_cast<uint8_t *>(&reached[0]);
        for(uint16_t i=0; i<sizeof(reached); i++) byte_arr[i+sizeof(walls_vertical)+sizeof(walls_horizontal)] = p[i];
    }

    void writeMazeData2Flash(void){
        const uint16_t WRITE_TARGET_BLOCK = 512;
        const uint16_t START_INDEX = WRITE_TARGET_BLOCK * DATA_FLASH_BLOCK_BYTE_SIZE;
        const uint16_t BLOCK_NUM = 10;
        const uint32_t LEN = DATA_FLASH_BLOCK_BYTE_SIZE;

        uint8_t write_val[BLOCK_NUM * LEN];
        serializeMazeData(&write_val[0]);

        //4*31+4*31+4*32 = 376byte
        // データフラッシュを10ブロック分イレイズ
        for(uint8_t i=0;i<BLOCK_NUM;i++){
            uint32_t index = START_INDEX + i*LEN;
            while(1){
                if(eraseCheckDataFlash(index, LEN) == false){
                    eraseDataFlash(index);
                };
                writeDataFlash(index, &write_val[i*LEN], LEN);
                uint8_t read_val[LEN];
                readDataFlash(index, &read_val[0], LEN);

                bool check_result = true;
                for(uint8_t j=0; j<LEN; j++){
                    if(read_val[j] != write_val[i*LEN+j]){
                        printfAsync("write error!\n");
                        check_result = false;
                    }
                }
                if(check_result == true) break;

            }
        }
    }
    void readMazeDataFromFlash(void){
        const uint16_t WRITE_TARGET_BLOCK = 512;
        const uint16_t START_INDEX = WRITE_TARGET_BLOCK * DATA_FLASH_BLOCK_BYTE_SIZE;
        const uint16_t BLOCK_NUM = 10;
        const uint32_t LEN = DATA_FLASH_BLOCK_BYTE_SIZE * BLOCK_NUM;


        uint8_t byte_arr[LEN];

        uint8_t* p;
        readDataFlash(START_INDEX, &byte_arr[0], LEN);

        p = reinterpret_cast<uint8_t *>(&walls_vertical[0]);
        for(uint16_t i=0; i<sizeof(walls_vertical); i++) p[i] = byte_arr[i];

        p = reinterpret_cast<uint8_t *>(&walls_horizontal[0]);
        for(uint16_t i=0; i<sizeof(walls_horizontal); i++) p[i] = byte_arr[i+sizeof(walls_vertical)];

        p = reinterpret_cast<uint8_t *>(&reached[0]);
        for(uint16_t i=0; i<sizeof(reached); i++) p[i] = byte_arr[i+sizeof(walls_vertical) +sizeof(walls_horizontal) ];

    }

};

}
