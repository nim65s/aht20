# `AHT20`

[![CI](https://github.com/nim65s/aht20/actions/workflows/ci.yml/badge.svg)](https://github.com/nim65s/aht20/actions/workflows/ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/nim65s/aht20/main.svg)](https://results.pre-commit.ci/latest/github/nim65s/aht20/main)

> An embedded rust `no_std` driver for the AHT20 temperature and humidity sensor, forked from @fmckeogh's [AHT20 crate](https://github.com/fmckeogh/aht20)

## Run embassy esp32 example

```
cargo run --example embassy-esp32 --target riscv32imc-unknown-none-elf -F esp32
```
