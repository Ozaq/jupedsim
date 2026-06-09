# AGENTS.md

## Project Overview

JuPedSim is a pedestrian dynamics simulation library. The repository combines C++20 simulation code, Python bindings, Python modules, examples, documentation, and vendored third-party dependencies.

## Repository Layout

- `libcommon/`: shared C++ utilities.
- `libsimulator/`: core C++ simulator implementation and operational models.
- `python_bindings_jupedsim/`: pybind11 bindings for the C++ library.
- `python_modules/`: Python packages, including `jupedsim` and `jupedsim_visualizer`.
- `systemtest/`: Python system tests.
- `performancetest/`: performance test code.
- `docs/`, `examples/`, `notebooks/`: user-facing documentation and examples.
- `third-party/`: vendored dependencies. Do not modify unless explicitly required.

## Build Requirements

- C++20-capable compiler.
- CMake `>= 3.22`.
- Python dependencies from `requirements.txt`.
- Git submodules initialized before full source builds.

Typical setup:

```bash
pip install -r requirements.txt
git submodule update --init
```

Build with setuptools:

```bash
pip install .
```

Manual CMake build:

```bash
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build
source build/environment
```

## Testing

Prefer targeted tests for the area changed.

Run Python tests directly when the native module is available in the environment:

```bash
pytest systemtest
pytest python_modules/jupedsim/tests
```

Run CMake-integrated tests after configuring with `-DBUILD_TESTS=ON`:

```bash
cmake --build build --target tests
```

Relevant CMake targets include:

- `tests`
- `pythontests`
- `unittests`
- `libsimulator-unittests`

## Formatting And Linting

Python formatting/linting is configured with Ruff in `pyproject.toml`.

```bash
ruff check .
ruff format .
```

Project conventions:

- Python line length: `80`.
- Python target version: `py312`.
- Use double quotes for Python strings.

C++ formatting uses `.clang-format`.

- C++ standard: `c++20`.
- Indentation: 4 spaces.
- Column limit: 100.
- Pointer/reference alignment is left.
- Include sorting is enabled.

When CMake is configured with `-DWITH_FORMAT=ON`, available targets include:

```bash
cmake --build build --target check-format
cmake --build build --target reformat
```

The format target expects `clang-format` version 21 unless `CLANG_FORMAT` points to a compatible executable.

When CMake is configured with `-DWITH_TIDY=ON`, run static analysis with:

```bash
cmake --build build --target check-tidy
```

The tidy target prefers `clang-tidy` version 21 and uses `.clang-tidy`.

## C++ Guidelines

- Keep code C++20-compliant.
- Follow Go-like visibility naming: public methods and fields use PascalCase; private and protected methods and fields use camelCase.
- Prefer small, localized changes.
- Follow existing naming and structure in the surrounding module.
- Add or update C++ unit tests when changing simulator behavior.
- Avoid introducing global compile flags; prefer target-specific or CMake generator-expression style consistent with existing code.

## Python Guidelines

- Keep public API changes deliberate and reflected in tests.
- Preserve compatibility between Python wrappers and native bindings.
- Add or update tests in `python_modules/jupedsim/tests` or `systemtest` depending on scope.
- Avoid editing generated, cache, or vendored files.

## Dependency And Vendor Policy

- Do not edit `third-party/` unless the task explicitly concerns vendored dependencies.
- Do not vendor new dependencies casually.
- If a dependency change is required, explain why and update the relevant build/package metadata.

## Documentation

Update docs, examples, or notebooks when behavior visible to users changes. For larger behavioral changes, prefer discussing the intended approach before implementation.

## Git Hygiene

- Do not revert unrelated user changes.
- Keep changes focused on the requested task.
- Do not commit unless explicitly asked.
