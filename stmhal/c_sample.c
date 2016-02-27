#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "portmodules.h"
#include <string.h>

STATIC mp_obj_t c_sample_set_callback(mp_obj_t callback_obj) {
    MP_STATE_PORT(c_sample_callback_obj) = callback_obj;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(c_sample_set_callback_obj, c_sample_set_callback);

STATIC mp_obj_t c_sample_call_callback(void) {
#if 1
    vstr_t vstr;
    vstr_init_len(&vstr, strlen("some_string"));
    strcpy(vstr.buf, "some_string");
    mp_obj_t obj = mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
#else
    mp_obj_t obj = mp_obj_new_str("some_string", strlen("some_string"), false);
#endif
    return mp_call_function_1(MP_STATE_PORT(c_sample_callback_obj), obj);
    //return mp_call_function_0(MP_STATE_PORT(c_sample_callback_obj));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(c_sample_call_callback_obj, c_sample_call_callback);

STATIC const mp_map_elem_t c_sample_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_c_sample) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_callback), (mp_obj_t)&c_sample_set_callback_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_call_callback), (mp_obj_t)&c_sample_call_callback_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_c_sample_globals, c_sample_globals_table);

const mp_obj_module_t mp_module_c_sample = {
    .base = { &mp_type_module },
    .name = MP_QSTR_c_sample,
    .globals = (mp_obj_dict_t*)&mp_module_c_sample_globals,
};

