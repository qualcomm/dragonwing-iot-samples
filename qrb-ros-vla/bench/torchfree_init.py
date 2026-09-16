"""Read LIBERO `.pruned_init` files (torch.save zip archives) WITHOUT torch.

A torch.save v2 file is a zip containing `archive/data.pkl` plus one raw
little-endian storage blob per tensor under `archive/data/<key>`.  The pickle
references `torch._utils._rebuild_tensor_v2` and `torch.<Dtype>Storage`, and
uses `persistent_load` to fetch the storage blobs.  We stub those out and
return numpy arrays.
"""
from __future__ import annotations

import io
import pickle
import zipfile

import numpy as np

_DTYPES = {
    "FloatStorage": np.dtype("<f4"),
    "DoubleStorage": np.dtype("<f8"),
    "HalfStorage": np.dtype("<f2"),
    "LongStorage": np.dtype("<i8"),
    "IntStorage": np.dtype("<i4"),
    "ShortStorage": np.dtype("<i2"),
    "CharStorage": np.dtype("<i1"),
    "ByteStorage": np.dtype("<u1"),
    "BoolStorage": np.dtype("?"),
}


class _StorageStub:
    def __init__(self, dtype: np.dtype):
        self.dtype = dtype


def _rebuild_tensor_v2(storage, storage_offset, size, stride, *args):
    arr = storage["data"]
    n = int(np.prod(size)) if len(size) else 1
    flat = arr[storage_offset: storage_offset + n]
    if stride and len(size) == len(stride):
        return np.lib.stride_tricks.as_strided(
            flat, shape=tuple(size),
            strides=tuple(s * arr.dtype.itemsize for s in stride),
        ).copy()
    return flat.reshape(tuple(size)).copy()


def _ordered_dict(*a, **k):
    import collections

    return collections.OrderedDict(*a, **k)


class _Unpickler(pickle.Unpickler):
    def __init__(self, f, zf: zipfile.ZipFile, prefix: str):
        super().__init__(f)
        self._zf = zf
        self._prefix = prefix

    def find_class(self, module, name):
        if module.startswith("torch") and name.endswith("Storage"):
            return _StorageStub(_DTYPES[name])
        if module == "torch._utils" and name == "_rebuild_tensor_v2":
            return _rebuild_tensor_v2
        if module == "collections" and name == "OrderedDict":
            return _ordered_dict
        return super().find_class(module, name)

    def persistent_load(self, pid):
        _tag, stub, key, _location, _numel = pid
        dtype = stub.dtype if isinstance(stub, _StorageStub) else np.dtype("<f4")
        for cand in (f"{self._prefix}/data/{key}", f"{self._prefix}/{key}"):
            try:
                raw = self._zf.read(cand)
                break
            except KeyError:
                continue
        else:
            raise KeyError(f"storage {key} not found in archive")
        return {"data": np.frombuffer(raw, dtype=dtype)}


def load_init_states(path: str) -> np.ndarray:
    with zipfile.ZipFile(path) as zf:
        names = zf.namelist()
        pkl = next(n for n in names if n.endswith("data.pkl"))
        prefix = pkl.rsplit("/", 1)[0]
        with zf.open(pkl) as f:
            obj = _Unpickler(io.BytesIO(f.read()), zf, prefix).load()
    return np.asarray(obj)


if __name__ == "__main__":
    import sys

    a = load_init_states(sys.argv[1])
    print(type(a), a.shape, a.dtype, float(a.min()), float(a.max()))
