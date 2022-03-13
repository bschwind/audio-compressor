# audio-compressor

A simple Rust port of sndfilter's compressor, which is extracted from the Chromium source.

Reference Implementation:

https://github.com/velipso/sndfilter/blob/56e0a9cfd91c43bd78048f54d24b84384b4fdb02/src/compressor.c

## Dependencies
- cargo
- rustc

## Build

```
$ cargo build --release
```

## Testing

```
$ cargo test
```

## Code Format

The formatting options currently use nightly-only options.

```
$ cargo +nightly fmt
```

## Code Linting

```
$ cargo clippy
```
