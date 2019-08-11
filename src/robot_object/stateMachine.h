#pragma once

#include <queue>
#include <memory>
#include "intent.h"

class BaseState{
public:
    Intent* intent;
    BaseState(Intent* intent_){
        nextState = nullptr;
        intent = intent_;
    }

    BaseState(){
        BaseState(nullptr);
        nextState = nullptr;
    }

    virtual void onStart(){}

    virtual void update(){};

    virtual bool isEnd(){return true;};


    std::unique_ptr<BaseState> nextState;


};


class StateMachine{
public:

    void update(){
        if(empty() == false){
            stateQueue.front()->update();
            if(stateQueue.front()->isEnd() == true){
                if(stateQueue.front()-> nextState != nullptr){
                    stateQueue.front()->nextState->onStart();
                    push(std::move(stateQueue.front()-> nextState));
                }
                stateQueue.pop();
            }
        }
        else
            return;

    }

    void push(std::unique_ptr<BaseState> &&state){
        if(empty()==true) state->onStart();
        stateQueue.push(std::move(state));

    }

    bool empty(){
        return stateQueue.empty();
    }

    void clear(){
        while(!stateQueue.empty()) stateQueue.pop();
    }
private:
    std::queue< std::unique_ptr<BaseState> > stateQueue;

};
