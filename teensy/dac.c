#include <stdio.h>
#include "Arduino.h"
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "dac.h"
#include "core/core_pins.h"

#define DAC_CHANNEL_1                      ((uint32_t)0x00000000)
#define DAC_CHANNEL_2                      ((uint32_t)0x00000010)
#define GPIO_PIN_4                 ((uint16_t)0x0010U)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020U)  /* Pin 5 selected    */




typedef enum {
    DAC_STATE_RESET,
    DAC_STATE_WRITE_SINGLE,
    DAC_STATE_BUILTIN_WAVEFORM,
    DAC_STATE_DMA_WAVEFORM,             // should be last enum since we use space beyond it
} pyb_dac_state_t;


typedef struct _pyb_dac_obj_t {
    mp_obj_base_t base;
    uint32_t dac_channel; // DAC_CHANNEL_1 or DAC_CHANNEL_2
//    const dma_descr_t *tx_dma_descr;
    uint16_t pin; // GPIO_PIN_4 or GPIO_PIN_5
    uint8_t bits; // 8 or 12
    uint8_t state;
} pyb_dac_obj_t;




/*
    \method write(value)
     Direct access to the DAC output (8 bit only at the moment).
*/

STATIC mp_obj_t pyb_dac_write(mp_obj_t self_in, mp_obj_t val) {

    pyb_dac_obj_t *self = self_in;
    if (self->state != DAC_STATE_WRITE_SINGLE) {
//        DAC_ChannelConfTypeDef config;
//        config.DAC_Trigger = DAC_TRIGGER_NONE;
//        config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
//        HAL_DAC_ConfigChannel(&DAC_Handle, &config, self->dac_channel);
        self->state = DAC_STATE_WRITE_SINGLE;
    }


    // DAC output is always 12-bit at the hardware level, and we provide support
    // for multiple bit "resolutions" simply by shifting the input value.
//    HAL_DAC_SetValue(&DAC_Handle, self->dac_channel, DAC_ALIGN_12B_R,
//        mp_obj_get_int(val) << (12 - self->bits));

//    HAL_DAC_Start(&DAC_Handle, self->dac_channel);

    analogWriteDAC0(mp_obj_get_int(val));
    return mp_const_none;
}


// create the dac object
// currently support either DAC1 on X5 (id = 1) or DAC2 on X6 (id = 2)

/// \classmethod \constructor(port)
/// Construct a new DAC object.
///
/// `port` can be a pin object, or an integer (1 or 2).
/// DAC(1) is on pin X5 and DAC(2) is on pin X6.

STATIC mp_obj_t pyb_dac_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // get pin/channel to output on
    mp_int_t dac_id;
    if (MP_OBJ_IS_INT(args[0])) {
        dac_id = mp_obj_get_int(args[0]);
    } else {
        const pin_obj_t *pin = pin_find(args[0]);
        if (pin == &pin_A4) {
            dac_id = 1;
        } else if (pin == &pin_A5) {
            dac_id = 2;
        } else {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "pin %q does not have DAC capabilities", pin->name));
        }
    }

    pyb_dac_obj_t *dac = m_new_obj(pyb_dac_obj_t);
    dac->base.type = &pyb_dac_type;

    if (dac_id == 1) {
        dac->pin = dac_id;
        } else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "DAC %d does not exist", dac_id));
    }

    // configure the peripheral
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
//    pyb_dac_init_helper(dac, n_args - 1, args + 1, &kw_args);

    // return object
    return dac;
}


STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_dac_write_obj, pyb_dac_write);



STATIC const mp_map_elem_t pyb_dac_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_write), (mp_obj_t)&pyb_dac_write_obj },

};

STATIC MP_DEFINE_CONST_DICT(pyb_dac_locals_dict, pyb_dac_locals_dict_table);

const mp_obj_type_t pyb_dac_type = {
    { &mp_type_type },
    .name = MP_QSTR_DAC,
    .make_new = pyb_dac_make_new,
    .locals_dict = (mp_obj_t)&pyb_dac_locals_dict,
};



