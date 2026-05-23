APP := ups_hat_controller
ROOT := $(abspath .)
BIN_SRC := $(ROOT)/target/release/$(APP)
ENV_SRC := $(ROOT)/config/ups-hat-controller.env
SERVICE_SRC := $(ROOT)/systemd/ups-hat-controller.service

BIN_DST := /usr/local/bin/$(APP)
ENV_DST := /etc/ups-hat-controller/ups-hat-controller.env
SERVICE_DST := /etc/systemd/system/ups-hat-controller.service

.PHONY: build release check fmt install-links uninstall-links status

build:
	cargo build

release:
	cargo build --release

check:
	cargo check

fmt:
	cargo fmt

install-links: release
	sudo install -d /etc/ups-hat-controller
	sudo ln -sfn "$(ENV_SRC)" "$(ENV_DST)"
	sudo ln -sfn "$(BIN_SRC)" "$(BIN_DST)"
	sudo ln -sfn "$(SERVICE_SRC)" "$(SERVICE_DST)"
	sudo systemctl daemon-reload

uninstall-links:
	sudo rm -f "$(ENV_DST)" "$(BIN_DST)" "$(SERVICE_DST)"
	sudo systemctl daemon-reload

status:
	@echo "ENV  -> $(ENV_DST)"
	@ls -l "$(ENV_DST)" || true
	@echo "BIN  -> $(BIN_DST)"
	@ls -l "$(BIN_DST)" || true
	@echo "UNIT -> $(SERVICE_DST)"
	@ls -l "$(SERVICE_DST)" || true
