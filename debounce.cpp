#include "debounce.h"
#include "myDelay.h"
#include "util.h"
extern uint8_t globalIndex;
extern Timer myTimer;



uint8_t contadorMask = 0;
void startButon(_sButton *myButton)
{   
    myButton->mask = 0;
    myButton[globalIndex].mask |= (1<<globalIndex) ;
    myButton->currentState = BUTTON_UP;
    myButton->stateInput = NO_EVENT;
    myButton->flagDetected = NOFLAG;
    myButton->timePressed = 0;
    myButton->timeDiff = 0;
    
}
 void StartHeratBeat(_sHearBeat *controlDeModos){
    controlDeModos->mask = 0x2AAAAAAA;
    controlDeModos->flagHearBeatBotonPres = 0;
    controlDeModos->flagCambiarHearBeat = 0;
    controlDeModos->flagBotonPresionado = 0;
    controlDeModos->flagModoActivo = 1;
    controlDeModos->flagModoCambiado = 1;
    controlDeModos->estadoPrograma = 5;
    controlDeModos->estadoHearBeat = 5;
 }



void buttonTask(_delay_t *timedebounce, _sButton *myButton, uint8_t statePulsadores,_sHearBeat *controlDeModos){
    
    if(delayRead(timedebounce)){
        myButton->stateInput = (_eEventInput)((statePulsadores & myButton->mask)>>globalIndex);
        switch (myButton->currentState){
            case BUTTON_UP:
                myButton->flagDetected = NOFLAG;
                if(myButton->stateInput==PRESSED)
                    myButton->currentState=BUTTON_FALLING;
            break;
            case BUTTON_FALLING:
                if(myButton->stateInput==PRESSED){
                    myButton->currentState=BUTTON_DOWN;
                    myButton->flagDetected = FALLINGFLAG;
                    myButton->timePressed = myTimer.read_ms();
                    //myButton->estadoPrograma++;
                    controlDeModos->flagBotonPresionado = true;
                }else{
                    myButton->currentState=BUTTON_UP;
                }
            break;
            case BUTTON_DOWN:
                ButtonDown(myButton,controlDeModos);
                if(myButton->stateInput==NOT_PRESSED)
                    myButton->currentState=BUTTON_RISING;
            break;
            case BUTTON_RISING:
                if(myButton->stateInput==NOT_PRESSED){
                        myButton->currentState=BUTTON_UP;
                        myButton->flagDetected = RISINGFLAG;
                        myButton->timeDiff = myTimer.read_ms()-myButton->timePressed;
                        controlDeModos->flagBotonPresionado = false;
                        ButtonRising(myButton,controlDeModos);
                        
                }else{
                    myButton->currentState=BUTTON_DOWN;
                }
            break;
            default:
                myButton->currentState=BUTTON_UP;
            break;
        }         
        
    }
    
}

void ButtonDown(_sButton *myButton,_sHearBeat *controlDeModos){

    if(((myTimer.read_ms() - myButton->timePressed) > 1000) && controlDeModos->flagBotonPresionado){
        controlDeModos->flagHearBeatBotonPres = true;
        controlDeModos->flagCambiarHearBeat = false;
        ControlHearbeat(controlDeModos);
    }

    if ((myTimer.read_ms() - myButton->timePressed) > 5000 && controlDeModos->flagBotonPresionado){
        controlDeModos->flagHearBeatBotonPres = false;
        controlDeModos->flagCambiarHearBeat = true;
        controlDeModos->flagModoActivo = false;
        ControlHearbeat(controlDeModos);

    }

}

void ButtonRising(_sButton *myButton,_sHearBeat *controlDeModos){
    if((myButton->timeDiff > 100) && (myButton->timeDiff <= 1000) && !controlDeModos->flagBotonPresionado ){
       controlDeModos->flagCambiarHearBeat = true;
       controlDeModos->flagHearBeatBotonPres = false;
       ControlHearbeat(controlDeModos);
       controlDeModos->estadoHearBeat++;

       if(controlDeModos->estadoHearBeat >5){
            controlDeModos->estadoHearBeat = 1;
       }
    
    }

    if((myButton->timeDiff > 1000) && (myButton->timeDiff <= 5000) && !controlDeModos->flagBotonPresionado ){
        controlDeModos->flagHearBeatBotonPres = false;
        controlDeModos->estadoPrograma = controlDeModos->estadoHearBeat;
        controlDeModos->flagModoCambiado = false;
        controlDeModos->flagModoActivo = true;
        ControlHearbeat(controlDeModos);
    }

}

void ControlHearbeat(_sHearBeat *controlDeModos){

if(controlDeModos->flagCambiarHearBeat){
    controlDeModos->mask = controlDeModos->MASK_SECUENCIA_MODO_OFF[controlDeModos->estadoHearBeat-2];
    controlDeModos->flagCambiarHearBeat = false;
    return;
}

if(controlDeModos->flagHearBeatBotonPres){
    controlDeModos->mask = controlDeModos->MASK_SECUENCIA_BOTON_PRESIONADO[controlDeModos->estadoHearBeat-2];
    controlDeModos->flagHearBeatBotonPres = false;
    return;
}
if(controlDeModos->flagModoActivo){
    controlDeModos->mask = controlDeModos->MASK_SECUENCIA_MODO_ON[controlDeModos->estadoHearBeat-2];
    return;
}
if(!controlDeModos->flagModoActivo){
    controlDeModos->mask = controlDeModos->MASK_SECUENCIA_MODO_OFF[controlDeModos->estadoHearBeat-2];

    return;
}

}