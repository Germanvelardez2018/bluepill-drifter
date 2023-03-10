#include "fsm.h"
#include "memory.h"
// Estado en ram
PRIVATE uint8_t FSM_DEVICE = FSM_CHECK_ONLY;



/* Inicializo la maquina de estados asociada a la aplicacion
*/
void fsm_init(){

    // hardware asociado

    // Estado inicial
   
    mem_s_get_fsm_state(&FSM_DEVICE);

}


// Obtienes valor en ram
uint8_t fsm_get_state(){
    return FSM_DEVICE;
}

void fsm_set_state(uint8_t state){
    // Seteo valor en ram
    FSM_DEVICE = state;
    // Seteo valor en flash
    mem_s_set_fsm_state(&FSM_DEVICE);


}
