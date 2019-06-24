/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "py/mperrno.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "driver/i2c.h"
#include "extmod/machine_i2c.h"

#if MICROPY_PY_MACHINE_I2C

#define WRITE_BIT                          0 /*!< I2C master write */
#define READ_BIT                           1  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

typedef struct _machine_hw_i2c_obj_t {
    mp_obj_base_t base;

    i2c_port_t i2c_port;
    uint32_t freq;
    uint32_t us_timeout;

    gpio_num_t sda;
    gpio_num_t scl;

    enum {
        MACHINE_HW_I2C_STATE_NONE,
        MACHINE_HW_I2C_STATE_INIT,
        MACHINE_HW_I2C_STATE_DEINIT
    } state;

    i2c_cmd_handle_t i2c_cmd;
} machine_hw_i2c_obj_t;

STATIC machine_hw_i2c_obj_t machine_hw_i2c_obj[2]; //only two hardware i2c

STATIC void machine_hw_i2c_deinit_internal(machine_hw_i2c_obj_t *self) {
    switch (i2c_driver_delete(self->i2c_port)) {  //uninstall driver
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, "invalid configuration");
            return;
    }

    int8_t pins[2] = {self->sda, self->scl}; //release i2c pin
    for (int i = 0; i < 2; i++) {
        if (pins[i] != -1) {
            gpio_pad_select_gpio(pins[i]);
            gpio_matrix_out(pins[i], SIG_GPIO_OUT_IDX, false, false);
            gpio_set_direction(pins[i], GPIO_MODE_INPUT);
        }
    }
}

STATIC void machine_hw_i2c_init_internal(machine_hw_i2c_obj_t *self, uint32_t freq, int8_t scl_io_num, int8_t sda_io_num)
{
    // if we're not initialized, then we're
    // implicitly 'changed', since this is the init routine
    bool changed = self->state != MACHINE_HW_I2C_STATE_INIT;
    esp_err_t ret;
    machine_hw_i2c_obj_t old_self = *self;

    if (freq != -1 && freq != self->freq) {
        self->freq = freq;
        changed = true;
    }

    if (sda_io_num != -1 && sda_io_num != self->sda) {
        self->sda = sda_io_num;
        changed = true;
    }

    if (scl_io_num != -1 && scl_io_num != self->scl) {
        self->scl = scl_io_num;
        changed = true;
    }

    if (changed) {
        if (self->state == MACHINE_HW_I2C_STATE_INIT) {
            self->state = MACHINE_HW_I2C_STATE_DEINIT;
            machine_hw_i2c_deinit_internal(&old_self);
        }
    } else {
        return; // no changes
    }

    //Initialize the i2c bus
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    i2c_param_config(self->i2c_port, &conf);
    ret = i2c_driver_install(self->i2c_port, conf.mode, 0, 0, 0);

    switch (ret) {
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, "invalid configuration");
            return;

        case ESP_FAIL:
            mp_raise_msg(&mp_type_OSError, "driver install error");
            return;
    }
    self->state = MACHINE_HW_I2C_STATE_INIT;
}

STATIC int machine_hw_i2c_obj_init_helper(mp_obj_base_t *obj, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_scl, ARG_sda, ARG_freq, ARG_timeout};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_scl, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_sda, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_freq, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 400000} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 255} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    machine_hw_i2c_obj_t *self = (machine_hw_i2c_obj_t *)obj;

    machine_hw_i2c_init_internal(
        self,
        args[ARG_freq].u_int,
        args[ARG_scl].u_obj == MP_OBJ_NULL ? GPIO_NUM_22 : machine_pin_get_id(args[ARG_scl].u_obj),
        args[ARG_sda].u_obj == MP_OBJ_NULL ? GPIO_NUM_23 : machine_pin_get_id(args[ARG_sda].u_obj));

    return 0;
}

STATIC int mp_hal_hw_i2c_start(machine_hw_i2c_obj_t *self) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = 0;
    self->i2c_cmd = cmd;
    ret = i2c_master_start(cmd);
    return (int)ret; // success
}

STATIC int mp_hal_hw_i2c_stop(machine_hw_i2c_obj_t *self) {
    i2c_master_stop(self->i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(self->i2c_port, self->i2c_cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(self->i2c_cmd);
    self->i2c_cmd = NULL;
    return (int)ret;
}

int mp_machine_hw_i2c_writeto(mp_obj_base_t *self_in, uint16_t addr, const uint8_t *src, size_t len, bool stop) {
    machine_hw_i2c_obj_t *self = (machine_hw_i2c_obj_t*)self_in;

    esp_err_t ret = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    if(src != NULL)
        ret |= i2c_master_write(cmd, (uint8_t *)src, len, ACK_CHECK_EN);
    ret |= i2c_master_stop(cmd);
    ret |= i2c_master_cmd_begin(self->i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return (int)ret;
}

int mp_machine_hw_i2c_readfrom(mp_obj_base_t *self_in, uint16_t addr, uint8_t *dest, size_t len, bool stop) {
    machine_hw_i2c_obj_t *self = (machine_hw_i2c_obj_t*)self_in;
    esp_err_t ret = 0;
    if (len == 0) {
        return 0;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        ret |= i2c_master_read(cmd, dest, len - 1, ACK_VAL);
    }
    ret |= i2c_master_read_byte(cmd, dest + len - 1, NACK_VAL);
    ret |= i2c_master_stop(cmd);
    ret |= i2c_master_cmd_begin(self->i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return (int)ret;
}

//read some bytes
int mp_machine_hw_i2c_read(mp_obj_base_t *self_in, uint8_t *dest, size_t len, bool nack) {
    machine_hw_i2c_obj_t *self = (machine_hw_i2c_obj_t*)self_in;
    esp_err_t ret = 0;
    if (len > 1) {
        ret |= i2c_master_read(self->i2c_cmd, dest, len - 1, ACK_VAL);
    }
    ret |= i2c_master_read_byte(self->i2c_cmd, dest + len - 1, NACK_VAL);
    return (int)ret;
}

//write some bytes
int mp_machine_hw_i2c_write(mp_obj_base_t *self_in, const uint8_t *src, size_t len) {
    machine_hw_i2c_obj_t *self = (machine_hw_i2c_obj_t*)self_in;
    esp_err_t ret = 0;
    ret = i2c_master_write(self->i2c_cmd, (uint8_t *)src, len, ACK_CHECK_EN);
    return (int)ret;
}

mp_obj_t machine_hw_i2c_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_port, ARG_freq, ARG_scl, ARG_sda};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_port,           MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = -1} }, //hard i2c port num  0 or 1
        { MP_QSTR_freq,           MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 400000} },
        { MP_QSTR_scl,            MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sda,            MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_hw_i2c_obj_t *self;
    if (args[ARG_port].u_int == 0) {
        self = &machine_hw_i2c_obj[0];
        self->i2c_port = 0;
    } else if(args[ARG_port].u_int == 1){
        self = &machine_hw_i2c_obj[1];
        self->i2c_port = 1;
    }
    else
    {
        mp_raise_msg(&mp_type_OSError, "invalid i2c port");
    }
    
    self->base.type = &machine_hw_i2c_type;

    machine_hw_i2c_init_internal(
        self,
        args[ARG_freq].u_int,
        args[ARG_scl].u_obj == MP_OBJ_NULL ? GPIO_NUM_22 : machine_pin_get_id(args[ARG_scl].u_obj),
        args[ARG_sda].u_obj == MP_OBJ_NULL ? GPIO_NUM_23 : machine_pin_get_id(args[ARG_sda].u_obj));

    return MP_OBJ_FROM_PTR(self);
}

STATIC const mp_machine_i2c_p_t mp_machine_hw_i2c_p = {
    .init = machine_hw_i2c_obj_init_helper,
    .start = (int(*)(mp_obj_base_t*))mp_hal_hw_i2c_start,
    .stop = (int(*)(mp_obj_base_t*))mp_hal_hw_i2c_stop,
    .read = mp_machine_hw_i2c_read,  //not include start send address ... stop stage
    .write = mp_machine_hw_i2c_write,
    .readfrom = mp_machine_hw_i2c_readfrom, //include atart send address read data stop stage.
    .writeto = mp_machine_hw_i2c_writeto,
};

const mp_obj_type_t machine_hw_i2c_type = {
    { &mp_type_type },
    .name = MP_QSTR_I2C,
    .make_new = machine_hw_i2c_make_new,
    .protocol = &mp_machine_hw_i2c_p,
    .locals_dict = (mp_obj_dict_t*)&mp_machine_soft_i2c_locals_dict, 
};

#endif // MICROPY_PY_MACHINE_I2C
