ARGS = ./roms/tests/panda.gba
EXE  = gba

SRC  = $(wildcard src/*.c)
OBJS = $(SRC:src/%.c=build/.src/%.o)
DEPS = $(SRC:src/%.c=build/.src/%.d)

CC   = gcc
CFLAGS = \
		 -g \
		 -pg \
		 -O0 \
		 -Wall \
		 -std=c99 \
		 -pedantic \
		 -MMD \
		 -Isrc \
		 `pkg-config --cflags sdl2`

LDFLAGS = \
		  -pg \
		  `pkg-config --libs sdl2`

all: info $(EXE)

info:
	@echo "CC:        $(CC)"
	@echo "CFLAGS:    $(CFLAGS)"
	@echo "LDFLAGS:   $(LDFLAGS)"
	@echo "SRC:       $(SRC)"

build/.src/%.o: src/%.c
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo [CC] $<

$(EXE): build/stamp $(OBJS)
	@$(CC) -o $@ $(OBJS) $(LDFLAGS)
	@echo [LD] $@

build/stamp:
	mkdir -p build/.src
	touch $@

run r: $(EXE)
	@./$(EXE) $(ARGS)

gdb: $(EXE)
	@echo GDB: $(EXE) $(ARGS)
	@gdb --args ./$(EXE) $(ARGS)

.PHONY: clean gdb run r
clean:
	rm -rf build
	rm -f $(EXE)

-include $(DEPS)
