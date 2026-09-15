################################################################################
# AXKU3 DDR4 bring-up example
#
#   make            Synthesize, implement and write build/axku3_uberddr4.bit
#   make synth      Stop after synthesis and its reports
#   make program    Program the board over JTAG with the built bitstream
#   make clean      Delete the build directory
#
# Vivado must be on PATH; 2022.2 is the recorded version. Override tool paths
# and build behaviour on the command line, for example:
#
#   make VIVADO=/opt/Xilinx/Vivado/2022.2/bin/vivado
#   make ALLOW_FAILING_TIMING=1     # write a bitstream that misses timing
#
# Windows has no make of its own; Vivado ships GNU Make 4.2.1 in its gnuwin/bin.
# That build runs every recipe through cmd.exe no matter which shell started it,
# so the recipes here stay within what cmd.exe and a POSIX shell both accept.
# A POSIX shell does not find vivado.bat by name alone, so a make that uses one
# on Windows needs VIVADO=vivado.bat or a full path to the launcher.
#
# Everything written stays in build/, which is not tracked by Git.
################################################################################

PROJECT = axku3_uberddr4
PART    = xcku3p-ffvb676-2-i
TOP     = $(PROJECT)
XDC     = $(PROJECT).xdc

TOP_VERILOG = $(PROJECT).v
RTL_DIR     = ../../rtl
ADDITIONAL_SOURCES = \
	$(RTL_DIR)/ddr4_top.v \
	$(RTL_DIR)/ddr4_controller.v \
	$(RTL_DIR)/ddr4_phy.v \
	$(RTL_DIR)/ddr4_prober.v \
	$(RTL_DIR)/phy/ddr4_phy_native.v \
	$(RTL_DIR)/phy/ddr4_phy_native_adapter.v \
	$(RTL_DIR)/phy/ddr4_phy_native_byte.v \
	$(RTL_DIR)/phy/ddr4_phy_native_reset.v

VIVADO   ?= vivado
BUILDDIR := build
BITSTREAM := $(BUILDDIR)/$(PROJECT).bit

# Vivado's Windows launcher exits 1 without printing anything when this is
# unset, which a stripped environment can do. A real value is never overridden,
# and nothing reads it on other hosts.
PROCESSOR_ARCHITECTURE ?= AMD64
export PROCESSOR_ARCHITECTURE

# Vivado is started from this directory so every path it is given stays
# relative, which keeps the recipes free of the path translation a Windows
# shell would otherwise need.
VIVADO_BATCH = $(VIVADO) -mode batch -nojournal -notrace

# A routed design that misses setup, hold or pulse width writes no bitstream.
ifneq ($(strip $(ALLOW_FAILING_TIMING)),)
BUILD_FLAGS += allow-failing-timing
endif
# Vivado rejects -tclargs with nothing after it, so pass it only when there is.
BUILD_TCLARGS = $(if $(strip $(BUILD_FLAGS)),-tclargs $(strip $(BUILD_FLAGS)),)

# Deleting a tree is the one step cmd.exe and a POSIX shell cannot spell alike.
# A make running recipes through cmd.exe reports it as the shell here.
ifeq ($(filter command.com cmd.exe,$(notdir $(SHELL))),)
REMOVE_BUILDDIR = rm -rf $(BUILDDIR)
else
REMOVE_BUILDDIR = if exist $(BUILDDIR) rmdir /s /q $(BUILDDIR)
endif

.PHONY: all
all: bitstream

.PHONY: bitstream
bitstream: $(BITSTREAM)

# Plain mkdir: the recipe only runs when the directory is absent, so the -p that
# cmd.exe would turn into a directory of that name is not needed.
$(BUILDDIR):
	mkdir $(BUILDDIR)

$(BITSTREAM): build.tcl $(TOP_VERILOG) $(ADDITIONAL_SOURCES) $(XDC) | $(BUILDDIR)
	$(VIVADO_BATCH) -source build.tcl -log $(BUILDDIR)/vivado.log $(BUILD_TCLARGS)

.PHONY: synth
synth: | $(BUILDDIR)
	$(VIVADO_BATCH) -source build.tcl -log $(BUILDDIR)/vivado_synth.log -tclargs synth $(BUILD_FLAGS)

.PHONY: program
program: $(BITSTREAM)
	$(VIVADO_BATCH) -source program.tcl -log $(BUILDDIR)/program.log -tclargs $(BITSTREAM)

.PHONY: clean
clean:
	$(REMOVE_BUILDDIR)
