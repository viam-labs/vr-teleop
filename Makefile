.DEFAULT_GOAL := help

-include Makefile.local

GO             := $(shell which go 2>/dev/null || echo /usr/local/go/bin/go)
BIN     := vr-teleop
HZ      ?= 90
OPENVR_VERSION ?= v2.5.1
OPENVR_SDK_DIR := openvr-sdk
OPENVR_SDK_URL := https://github.com/ValveSoftware/openvr/archive/refs/tags/$(OPENVR_VERSION).tar.gz

.PHONY: help
help:
	@echo 'VR Teleop'
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Available targets:'
	@echo '  deps     - Download OpenVR SDK and tidy Go modules'
	@echo '  build    - Build the binary'
	@echo '  run      - Build and run'
	@echo '  clean    - Remove SDK and compiled binary'
	@echo '  help     - Show this help message'

.PHONY: deps
deps:
	@if [ ! -f $(OPENVR_SDK_DIR)/headers/openvr_capi.h ]; then \
		echo '[teleop] Downloading OpenVR SDK $(OPENVR_VERSION)...'; \
		mkdir -p $(OPENVR_SDK_DIR); \
		curl -fsSL $(OPENVR_SDK_URL) \
			| tar -xz --strip-components=1 -C $(OPENVR_SDK_DIR) \
			    --wildcards \
			    '*/headers/openvr_capi.h' \
			    '*/lib/linux64/libopenvr_api.so'; \
		echo '[teleop] OpenVR SDK ready at $(OPENVR_SDK_DIR)'; \
	else \
		echo '[teleop] OpenVR SDK already present ($(OPENVR_SDK_DIR))'; \
	fi
	@if [ ! -f go.mod ]; then \
		echo '[teleop] Initializing Go module...'; \
		$(GO) mod init github.com/viam-labs/vr-teleop; \
	fi
	@echo '[teleop] Tidying Go modules...'
	@$(GO) mod tidy

.PHONY: build
build: deps
	@echo '[teleop] Building...'
	@$(GO) build -o $(BIN) .
	@echo '[teleop] Built: $(BIN)'

.PHONY: run
run: build
	@echo '[teleop] Starting at $(HZ) Hz...'
	@LD_LIBRARY_PATH=$(CURDIR)/$(OPENVR_SDK_DIR)/lib/linux64:$$LD_LIBRARY_PATH \
		./$(BIN) --hz $(HZ) $(RUN_ARGS)

.PHONY: clean
clean:
	@rm -rf $(OPENVR_SDK_DIR) $(BIN)
	@echo '[teleop] Cleaned.'
