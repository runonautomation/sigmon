#pragma once
/* Host stub: register writes go nowhere; the tests exercise logic, not pins. */
#define REG_WRITE(reg, val) ((void)(reg), (void)(val))
