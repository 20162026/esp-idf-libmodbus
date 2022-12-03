set(src "libmodbus/src/modbus.c"
        "libmodbus/src/modbus.h"
        "libmodbus/src/modbus-data.c"
        "libmodbus/src/modbus-private.h"
        "libmodbus/src/modbus-rtu.c"
        "libmodbus/src/modbus-rtu.h"
        "libmodbus/src/modbus-rtu-private.h"
        "libmodbus/src/modbus-tcp.c"
        "libmodbus/src/modbus-tcp.h"
        "libmodbus/src/modbus-tcp-private.h"
        "include/modbus-version.h")

set(include "libmodbus/src" "include")

idf_component_register( SRCS ${src}
                        INCLUDE_DIRS ${include}
                        )
                    