all: bt_simulator ble_arch_sim

bt_simulator: bt_simulator.c
	$(CC) $(CFLAGS) bt_simulator.c -o bt_simulator

ble_arch_sim: ble_arch_sim.c
	$(CC) $(CFLAGS) ble_arch_sim.c -o ble_arch_sim

run: bt_simulator
	./bt_simulator

run_ble: ble_arch_sim
	./ble_arch_sim

benchmark: ble_arch_sim
	./ble_arch_sim --benchmark

auto: bt_simulator
	./bt_simulator --auto

test: bt_simulator
	./bt_simulator --test 6

clean:
	rm -f bt_simulator ble_arch_sim

.PHONY: all run run_ble benchmark auto test clean
