#include <stdlib.h>
#include <stdio.h>

#include "cpu.h"
#include "bus.h"

int main(int argc, char **argv)
{
    if (argc < 2) {
        puts("usage: gba ROM");
        return 1;
    }

    Bus *bus = bus_init(argv[1]);
    Cpu *cpu = cpu_init(bus);

    while (1) {
        int n = cpu_step(cpu);
        if (n == 0)
            break;
    }

    // do we really need to free memory now?
    free(bus);
    free(cpu);

	return 0;
}
