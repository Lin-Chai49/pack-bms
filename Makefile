CC = cc
CFLAGS = -std=c99 -Wall -Wextra -O2 -Iinclude
# Host test: ASan + UBSan catch NULL, overflow, INT32_MIN negation.
TESTFLAGS = -std=c99 -Wall -Wextra -O1 -g -Iinclude -fsanitize=address,undefined

.PHONY: test clean

test: host/bms_test
	./host/bms_test

host/bms_test: host/main_host.c src/bms.c src/bms_regs.c include/bms.h include/bms_hal.h include/bms_regs.h
	$(CC) $(TESTFLAGS) -o $@ host/main_host.c src/bms.c src/bms_regs.c

clean:
	rm -f host/bms_test
