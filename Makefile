# ==============================================================================
#  Makefile - WiperControl_SWC host build
#
#  Targets:
#    all           build the non-interactive test bench  -> wiper_sim
#    run           build and run the test bench
#    cli           build and start the interactive simulator -> wiper_cli
#    validate-xml  check WiperControl_SWC.arxml for well-formedness
#    check         run + validate-xml (full host-side validation)
#    clean         remove all build artefacts
#
#  Note on -Irte_mock:
#    WiperControl.c includes the RTE headers Std_Types.h, Rte_Type.h and
#    Rte_WiperControl_SWC.h, which are produced by the RTE generator on target.
#    rte_mock/ holds host-side stand-ins with matching signatures so the SWC
#    compiles unmodified. The mock FUNCTIONS live in test_runner.c / cli_sim.c,
#    not in the headers.
# ==============================================================================

CC       := gcc
CFLAGS   := -Wall -Wextra -std=c99
INCLUDES := -Irte_mock

ARXML    := WiperControl_SWC.arxml
SWC_SRC  := WiperControl.c
SWC_HDR  := WiperControl.h

TEST_SRC := test_runner.c
TEST_BIN := wiper_sim

CLI_SRC  := cli_sim.c
CLI_BIN  := wiper_cli

.PHONY: all run cli validate-xml check clean

all: $(TEST_BIN)

$(TEST_BIN): $(TEST_SRC) $(SWC_SRC) $(SWC_HDR)
	$(CC) $(CFLAGS) $(INCLUDES) $(TEST_SRC) $(SWC_SRC) -o $(TEST_BIN)

$(CLI_BIN): $(CLI_SRC) $(SWC_SRC) $(SWC_HDR)
	$(CC) $(CFLAGS) $(INCLUDES) $(CLI_SRC) $(SWC_SRC) -o $(CLI_BIN)

# Build and execute the non-interactive test bench.
run: $(TEST_BIN)
	./$(TEST_BIN)

# Build and start the interactive simulator.
cli: $(CLI_BIN)
	./$(CLI_BIN)

# Check the ARXML for well-formedness.
# xmllint is the primary tool; python3 is a fallback for hosts where libxml2
# is not installed (the result is equivalent for well-formedness checking).
validate-xml:
	@if command -v xmllint >/dev/null 2>&1; then \
		echo "[validate-xml] running xmllint on $(ARXML)"; \
		xmllint --noout $(ARXML) && echo "[validate-xml] OK - $(ARXML) is well-formed"; \
	else \
		echo "[validate-xml] xmllint not found, falling back to python3"; \
		python3 -c "import xml.etree.ElementTree as ET; ET.parse('$(ARXML)'); print('[validate-xml] OK - $(ARXML) is well-formed')"; \
	fi

# Everything the host side can check on its own.
check: run validate-xml
	@echo "[check] host-side validation complete"

clean:
	rm -f $(TEST_BIN) $(CLI_BIN) *.o
