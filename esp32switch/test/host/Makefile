# Host-side logic tests: compiles the real switch_ctl.c against stub ESP-IDF
# headers so the parsing and state machine can be exercised without hardware.
#   make && ./test_switch
CFLAGS = -std=gnu11 -Wall -Wextra -Wno-unused-parameter -O1 -I stub -I ../../main

test_switch: test_switch.c ../../main/switch_ctl.c stub/stubs.c
	$(CC) $(CFLAGS) $^ -o $@

.PHONY: run clean
run: test_switch
	./test_switch

clean:
	rm -f test_switch
