# Developer Documentation

## Generate python stubs

Navigate to the folder with `pyproject.toml`
```bash
cd vda5050_core
```

Run the `pybind11-stubgen` command to generate `.pyi` files.

```bash
uv run pybind11-stubgen vda5050_core -o ./python
```
